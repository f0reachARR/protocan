#include "protocan/descriptor_parser.hpp"

#include "protocan/descriptor.pb.h"

namespace protocan
{

namespace
{

ParsedField convert_field(const protocan::FieldDescriptor & pb_field)
{
  ParsedField f;
  f.name = pb_field.name();
  f.type = static_cast<uint8_t>(pb_field.type());
  f.offset = pb_field.offset();
  f.size = pb_field.size();
  f.ros2_field = pb_field.ros2_field();
  return f;
}

ParsedMessage convert_message(const protocan::MessageDescriptor & pb_msg)
{
  ParsedMessage m;
  m.ros2_msg_type = pb_msg.ros2_msg_type();
  m.payload_size = pb_msg.payload_size();
  m.fields.reserve(pb_msg.fields_size());
  for (int i = 0; i < pb_msg.fields_size(); ++i) {
    m.fields.push_back(convert_field(pb_msg.fields(i)));
  }
  return m;
}

}  // namespace

bool parse_descriptor(const uint8_t * blob, size_t len, ParsedDescriptor & out)
{
  protocan::NodeDescriptor pb_desc;
  if (!pb_desc.ParseFromArray(blob, static_cast<int>(len))) {
    return false;
  }

  out.schema_hash = pb_desc.schema_hash();
  out.node_type_name = pb_desc.node_type_name();
  out.ros2_namespace = pb_desc.ros2_namespace();

  // Topics
  out.topics.clear();
  out.topics.reserve(pb_desc.topics_size());
  for (int i = 0; i < pb_desc.topics_size(); ++i) {
    const auto & pb_topic = pb_desc.topics(i);
    ParsedTopic t;
    t.index = pb_topic.index();
    t.name = pb_topic.name();
    t.is_tx = pb_topic.is_tx();
    t.periodic = pb_topic.periodic();
    t.priority = pb_topic.priority();
    if (pb_topic.has_message()) {
      t.message = convert_message(pb_topic.message());
    }
    out.topics.push_back(std::move(t));
  }

  // Services
  out.services.clear();
  out.services.reserve(pb_desc.services_size());
  for (int i = 0; i < pb_desc.services_size(); ++i) {
    const auto & pb_svc = pb_desc.services(i);
    ParsedService s;
    s.index = pb_svc.index();
    s.name = pb_svc.name();
    s.ros2_srv_type = pb_svc.ros2_srv_type();
    if (pb_svc.has_request()) {
      s.request = convert_message(pb_svc.request());
    }
    if (pb_svc.has_response()) {
      s.response = convert_message(pb_svc.response());
    }
    out.services.push_back(std::move(s));
  }

  // Params
  out.params.clear();
  out.params.reserve(pb_desc.params_size());
  for (int i = 0; i < pb_desc.params_size(); ++i) {
    const auto & pb_param = pb_desc.params(i);
    ParsedParam p;
    p.index = pb_param.index();
    p.name = pb_param.name();
    p.type = static_cast<uint8_t>(pb_param.type());
    p.read_only = pb_param.read_only();
    out.params.push_back(std::move(p));
  }

  return true;
}

bool validate_descriptor(const ParsedDescriptor & desc)
{
  // node_type_name は必須
  if (desc.node_type_name.empty()) {
    return false;
  }

  // トピックのインデックスは連続しているか確認
  for (size_t i = 0; i < desc.topics.size(); ++i) {
    if (desc.topics[i].index != static_cast<uint32_t>(i)) {
      return false;
    }
    // ペイロードサイズが 0 は無効（フィールドがあるなら）
    if (!desc.topics[i].message.fields.empty() && desc.topics[i].message.payload_size == 0) {
      return false;
    }
  }

  // サービスのインデックス確認
  for (size_t i = 0; i < desc.services.size(); ++i) {
    if (desc.services[i].index != static_cast<uint32_t>(i)) {
      return false;
    }
  }

  // パラメータのインデックス確認
  for (size_t i = 0; i < desc.params.size(); ++i) {
    if (desc.params[i].index != static_cast<uint32_t>(i)) {
      return false;
    }
  }

  return true;
}

bool check_payload_compatibility(const ParsedMessage & a, const ParsedMessage & b)
{
  // descriptor_spec.md §3.5:
  // 1. payload_size が一致
  if (a.payload_size != b.payload_size) {
    return false;
  }

  // 2. fields 数が一致
  if (a.fields.size() != b.fields.size()) {
    return false;
  }

  // 3. 各フィールドの type, offset, size が全て一致
  for (size_t i = 0; i < a.fields.size(); ++i) {
    if (
      a.fields[i].type != b.fields[i].type || a.fields[i].offset != b.fields[i].offset ||
      a.fields[i].size != b.fields[i].size) {
      return false;
    }
  }

  // フィールド名や ros2_field の一致は要求しない
  return true;
}

}  // namespace protocan
