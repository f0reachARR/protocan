#include "fibril_transport_common/dynamic_ros_interface.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ros_babel_fish/exceptions/babel_fish_exception.hpp>
#include <ros_babel_fish/idl/type_support.hpp>
#include <ros_babel_fish/messages/array_message.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>
#include <ros_babel_fish/messages/value_message.hpp>
#include <sstream>
#include <stdexcept>

namespace fibril_transport_common
{

FieldType messageToFieldType(const ros_babel_fish::Message & message)
{
  if (message.type() == ros_babel_fish::MessageTypes::Compound) {
    return CompoundFieldTypeInfo::from(message);
  } else if (message.type() == ros_babel_fish::MessageTypes::Array) {
    return ArrayFieldTypeInfo::from(message);
  } else {
    switch (message.type()) {
      case ros_babel_fish::MessageType::Bool:
        return PrimitiveType::Bool;
      case ros_babel_fish::MessageType::Int8:
        return PrimitiveType::Int8;
      case ros_babel_fish::MessageType::UInt8:
        return PrimitiveType::UInt8;
      case ros_babel_fish::MessageType::Int16:
        return PrimitiveType::Int16;
      case ros_babel_fish::MessageType::UInt16:
        return PrimitiveType::UInt16;
      case ros_babel_fish::MessageType::Int32:
        return PrimitiveType::Int32;
      case ros_babel_fish::MessageType::UInt32:
        return PrimitiveType::UInt32;
      case ros_babel_fish::MessageType::Int64:
        return PrimitiveType::Int64;
      case ros_babel_fish::MessageType::UInt64:
        return PrimitiveType::UInt64;
      case ros_babel_fish::MessageType::Float:
        return PrimitiveType::Float;
      case ros_babel_fish::MessageType::Double:
        return PrimitiveType::Double;
      default:
        throw std::runtime_error("Unknown ROS type: " + std::to_string(message.type()));
    }
  }
}

ArrayFieldTypeInfo::Ptr ArrayFieldTypeInfo::from(const ros_babel_fish::Message & message)
{
  const auto & array_msg = dynamic_cast<const ros_babel_fish::ArrayMessageBase &>(message);

  ArrayFieldTypeInfo::Ptr info = std::make_shared<ArrayFieldTypeInfo>();

  info->is_fixed = array_msg.isFixedSize();
  info->max_array_size = array_msg.maxSize();
  info->array_size = array_msg.size();

  info->element_type_info = messageToFieldType(array_msg);

  return info;
}

CompoundFieldTypeInfo::Ptr CompoundFieldTypeInfo::from(const ros_babel_fish::Message & message)
{
  const auto & compound_msg = dynamic_cast<const ros_babel_fish::CompoundMessage &>(message);

  CompoundFieldTypeInfo::Ptr info = std::make_shared<CompoundFieldTypeInfo>();

  info->type_name = compound_msg.name();

  return info;
}

DynamicRosInterface::DynamicRosInterface()
: babel_fish_(std::make_unique<ros_babel_fish::BabelFish>())
{
}

DynamicRosInterface::~DynamicRosInterface() = default;

// ========== Codegen side implementation ==========

ros_babel_fish::MessageTypeSupport::ConstSharedPtr DynamicRosInterface::getMessageTypeSupport(
  const std::string & type_name) const
{
  try {
    return babel_fish_->get_message_type_support(type_name);
  } catch (const ros_babel_fish::BabelFishException & e) {
    throw std::runtime_error(
      "Failed to get message type support for '" + type_name + "': " + e.what());
  } catch (const ament_index_cpp::PackageNotFoundError & e) {
    throw std::runtime_error(
      "Failed to get message type support for '" + type_name + "': " + e.what());
  }
}

ros_babel_fish::ServiceTypeSupport::ConstSharedPtr DynamicRosInterface::getServiceTypeSupport(
  const std::string & type_name) const
{
  try {
    return babel_fish_->get_service_type_support(type_name);
  } catch (const ros_babel_fish::BabelFishException & e) {
    throw std::runtime_error(
      "Failed to get service type support for '" + type_name + "': " + e.what());
  } catch (const ament_index_cpp::PackageNotFoundError & e) {
    throw std::runtime_error(
      "Failed to get service type support for '" + type_name + "': " + e.what());
  }
}

ros_babel_fish::MessageMembersIntrospection DynamicRosInterface::getMessageMembersIntrospection(
  ros_babel_fish::MessageTypeSupport::ConstSharedPtr type_support) const
{
  return ros_babel_fish::MessageMembersIntrospection(*type_support);
}

ros_babel_fish::MessageMembersIntrospection
DynamicRosInterface::getServiceRequestMembersIntrospection(
  ros_babel_fish::ServiceTypeSupport::ConstSharedPtr type_support) const
{
  return type_support->request();
}

ros_babel_fish::MessageMembersIntrospection
DynamicRosInterface::getServiceResponseMembersIntrospection(
  ros_babel_fish::ServiceTypeSupport::ConstSharedPtr type_support) const
{
  return type_support->response();
}

bool DynamicRosInterface::validateFieldPath(
  const ros_babel_fish::MessageMembersIntrospection members, const std::string & field_path) const
{
  try {
    // Create a temporary message to validate the field path
    auto msg = createMessage(members);

    if (field_path.empty()) {
      return true;  // Root level is always valid
    }

    const auto paths = splitFieldPath(field_path);
    navigateToField(*msg, paths);
    return true;
  } catch (...) {
    return false;
  }
}

FieldTypeInfo DynamicRosInterface::getFieldTypeInfo(
  const ros_babel_fish::MessageMembersIntrospection members, const std::string & field_path) const
{
  auto msg = createMessage(members);

  if (field_path.empty()) {
    throw std::runtime_error("Field path cannot be empty for getFieldTypeInfo");
  }

  const auto paths = splitFieldPath(field_path);
  const auto & field = navigateToField(*msg, paths);

  // Extract field name (last component of path)
  std::string field_name = paths.back();

  FieldTypeInfo info;
  info.field_name = field_name;
  info.type_info = messageToFieldType(field);

  return info;
}

std::vector<FieldTypeInfo> DynamicRosInterface::getFieldList(
  const ros_babel_fish::MessageMembersIntrospection members, const std::string & field_path) const
{
  auto msg = createMessage(members);

  const ros_babel_fish::CompoundMessage * target_msg = msg.get();

  // Navigate to the specified hierarchy if path is not empty
  if (!field_path.empty()) {
    const auto paths = splitFieldPath(field_path);
    const auto & field = navigateToField(*msg, paths);

    // Ensure the target is a compound message
    if (field.type() != ros_babel_fish::MessageTypes::Compound) {
      throw std::runtime_error(
        "Field path '" + field_path + "' does not point to a compound message");
    }

    target_msg = &dynamic_cast<const ros_babel_fish::CompoundMessage &>(field);
  }

  // Get all field names
  auto keys = target_msg->keys();
  std::vector<FieldTypeInfo> result;

  for (const auto & key : keys) {
    const auto & field = (*target_msg)[key];

    FieldTypeInfo info;
    info.field_name = key;
    info.type_info = messageToFieldType(field);

    result.push_back(info);
  }

  return result;
}

// ========== Master side implementation ==========

ros_babel_fish::CompoundMessage::SharedPtr DynamicRosInterface::createMessage(
  const std::string & type_name) const
{
  try {
    return createMessage(getMessageTypeSupport(type_name));
  } catch (const ros_babel_fish::BabelFishException & e) {
    throw std::runtime_error("Failed to create message of type '" + type_name + "': " + e.what());
  }
}

ros_babel_fish::CompoundMessage::SharedPtr DynamicRosInterface::createMessage(
  const ros_babel_fish::MessageTypeSupport::ConstSharedPtr & type_support) const
{
  return createMessage(ros_babel_fish::MessageMembersIntrospection(*type_support));
}

ros_babel_fish::CompoundMessage::SharedPtr DynamicRosInterface::createMessage(
  const ros_babel_fish::MessageMembersIntrospection type_support) const
{
  try {
    return ros_babel_fish::CompoundMessage::make_shared(type_support);
  } catch (const ros_babel_fish::BabelFishException & e) {
    std::string message_name(type_support->message_name_);
    throw std::runtime_error(
      "Failed to create message of type '" + message_name + "': " + e.what());
  }
}

ros_babel_fish::BabelFishServiceClient::SharedPtr DynamicRosInterface::createServiceClient(
  rclcpp::Node & node, const std::string & service_name, const std::string & service_type) const
{
  try {
    return babel_fish_->create_service_client(node, service_name, service_type);
  } catch (const ros_babel_fish::BabelFishException & e) {
    throw std::runtime_error(
      "Failed to create service client for '" + service_name + "' (" + service_type +
      "): " + e.what());
  }
}

ros_babel_fish::BabelFishService::SharedPtr DynamicRosInterface::createService(
  rclcpp::Node & node, const std::string & service_name, const std::string & service_type,
  ros_babel_fish::AnyServiceCallback callback) const
{
  try {
    return babel_fish_->create_service(node, service_name, service_type, callback);
  } catch (const ros_babel_fish::BabelFishException & e) {
    throw std::runtime_error(
      "Failed to create service '" + service_name + "' (" + service_type + "): " + e.what());
  }
}

// ========== Private helper methods ==========

ros_babel_fish::Message & DynamicRosInterface::navigateToField(
  ros_babel_fish::CompoundMessage & message, const std::vector<std::string> & paths) const
{
  ros_babel_fish::Message * current = &message;

  for (const auto & part : paths) {
    if (current->type() != ros_babel_fish::MessageTypes::Compound) {
      throw std::runtime_error("Cannot navigate through non-compound message at '" + part + "'");
    }

    auto & compound = dynamic_cast<ros_babel_fish::CompoundMessage &>(*current);

    if (!compound.containsKey(part)) {
      throw std::runtime_error("Field '" + part + "' not found in message");
    }

    current = &compound[part];
  }

  return *current;
}

const ros_babel_fish::Message & DynamicRosInterface::navigateToField(
  const ros_babel_fish::CompoundMessage & message, const std::vector<std::string> & paths) const
{
  const ros_babel_fish::Message * current = &message;

  for (const auto & part : paths) {
    if (current->type() != ros_babel_fish::MessageTypes::Compound) {
      throw std::runtime_error("Cannot navigate through non-compound message at '" + part + "'");
    }

    const auto & compound = dynamic_cast<const ros_babel_fish::CompoundMessage &>(*current);

    if (!compound.containsKey(part)) {
      throw std::runtime_error("Field '" + part + "' not found in message");
    }

    current = &compound[part];
  }

  return *current;
}

std::vector<std::string> DynamicRosInterface::splitFieldPath(const std::string & field_path) const
{
  std::vector<std::string> parts;
  std::stringstream ss(field_path);
  std::string part;

  while (std::getline(ss, part, '.')) {
    if (!part.empty()) {
      parts.push_back(part);
    }
  }

  return parts;
}

ros_babel_fish::Message & DynamicRosInterface::navigateToField(
  ros_babel_fish::CompoundMessage & message, const std::string & field_path) const
{
  return navigateToField(message, splitFieldPath(field_path));
}

const ros_babel_fish::Message & DynamicRosInterface::navigateToField(
  const ros_babel_fish::CompoundMessage & message, const std::string & field_path) const
{
  return navigateToField(message, splitFieldPath(field_path));
}

// ========== Template specializations for setFieldValue/getFieldValue ==========

template <>
void DynamicRosInterface::setFieldValue<bool>(
  ros_babel_fish::CompoundMessage & message, const std::string & field_path,
  const bool & value) const
{
  auto & field = navigateToField(message, field_path);
  field = value;
}

template <>
void DynamicRosInterface::setFieldValue<int32_t>(
  ros_babel_fish::CompoundMessage & message, const std::string & field_path,
  const int32_t & value) const
{
  auto & field = navigateToField(message, field_path);
  field = value;
}

template <>
void DynamicRosInterface::setFieldValue<uint32_t>(
  ros_babel_fish::CompoundMessage & message, const std::string & field_path,
  const uint32_t & value) const
{
  auto & field = navigateToField(message, field_path);
  field = value;
}

template <>
void DynamicRosInterface::setFieldValue<int64_t>(
  ros_babel_fish::CompoundMessage & message, const std::string & field_path,
  const int64_t & value) const
{
  auto & field = navigateToField(message, field_path);
  field = value;
}

template <>
void DynamicRosInterface::setFieldValue<float>(
  ros_babel_fish::CompoundMessage & message, const std::string & field_path,
  const float & value) const
{
  auto & field = navigateToField(message, field_path);
  field = value;
}

template <>
void DynamicRosInterface::setFieldValue<double>(
  ros_babel_fish::CompoundMessage & message, const std::string & field_path,
  const double & value) const
{
  auto & field = navigateToField(message, field_path);
  field = value;
}

template <>
bool DynamicRosInterface::getFieldValue<bool>(
  const ros_babel_fish::CompoundMessage & message, const std::string & field_path) const
{
  const auto & field = navigateToField(message, field_path);
  return field.value<bool>();
}

template <>
int32_t DynamicRosInterface::getFieldValue<int32_t>(
  const ros_babel_fish::CompoundMessage & message, const std::string & field_path) const
{
  const auto & field = navigateToField(message, field_path);
  return field.value<int32_t>();
}

template <>
uint32_t DynamicRosInterface::getFieldValue<uint32_t>(
  const ros_babel_fish::CompoundMessage & message, const std::string & field_path) const
{
  const auto & field = navigateToField(message, field_path);
  return field.value<uint32_t>();
}

template <>
int64_t DynamicRosInterface::getFieldValue<int64_t>(
  const ros_babel_fish::CompoundMessage & message, const std::string & field_path) const
{
  const auto & field = navigateToField(message, field_path);
  return field.value<int64_t>();
}

template <>
float DynamicRosInterface::getFieldValue<float>(
  const ros_babel_fish::CompoundMessage & message, const std::string & field_path) const
{
  const auto & field = navigateToField(message, field_path);
  return field.value<float>();
}

template <>
double DynamicRosInterface::getFieldValue<double>(
  const ros_babel_fish::CompoundMessage & message, const std::string & field_path) const
{
  const auto & field = navigateToField(message, field_path);
  return field.value<double>();
}

}  // namespace fibril_transport_common
