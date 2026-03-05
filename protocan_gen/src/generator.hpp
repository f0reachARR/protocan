#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <google/protobuf/compiler/code_generator.h>
#include <google/protobuf/descriptor.h>

namespace protocan_gen
{

enum class RpcKind { TX_TOPIC, RX_TOPIC, SERVICE, PARAM_RW, PARAM_RO };

struct FieldInfo
{
  std::string name;          // proto field name
  std::string cpp_type;      // C++ type (e.g. "float", "uint8_t")
  int         field_type_enum;  // protocan::FieldType value
  int         packed_size;   // byte count in packed binary
  int         offset;        // byte offset within message packed binary
  std::string ros2_field;    // ROS2 field path (may be empty)
};

struct MessageInfo
{
  std::string           proto_name;    // CamelCase message name
  std::string           ros2_msg_type; // may be empty
  int                   packed_size;
  std::vector<FieldInfo> fields;
  bool                  is_empty;      // true if google.protobuf.Empty
};

struct RpcInfo
{
  RpcKind     kind;
  std::string proto_name;   // original CamelCase RPC name
  std::string snake_name;   // snake_case (from proto_name or ros2_name override)
  int         index;        // per-kind 0-based index
  bool        periodic;
  uint32_t    priority;
  bool        read_only;
  std::string ros2_name;     // from option (may be empty)
  std::string ros2_srv_type; // from option (may be empty)
  int         request_msg_idx;   // index into ServiceInfo::messages (-1 = Empty)
  int         response_msg_idx;  // index into ServiceInfo::messages (-1 = Empty)
};

struct ServiceInfo
{
  std::string              proto_service_name;  // e.g. "BLDCMotor"
  std::string              snake_name;          // e.g. "bldc_motor"
  std::string              package_name;        // e.g. "bldc_motor" (may be empty)
  std::string              ros2_namespace;
  std::vector<MessageInfo> messages;            // unique messages (no Empty)
  std::vector<RpcInfo>     rpcs;
  int                      tx_topic_count = 0;
  int                      rx_topic_count = 0;
  int                      service_count  = 0;
  int                      param_count    = 0;
};

class ProtocAnGenerator : public google::protobuf::compiler::CodeGenerator
{
public:
  bool Generate(
    const google::protobuf::FileDescriptor *          file,
    const std::string &                               parameter,
    google::protobuf::compiler::GeneratorContext *    context,
    std::string *                                     error) const override;

  uint64_t GetSupportedFeatures() const override
  {
    return FEATURE_PROTO3_OPTIONAL;
  }
};

}  // namespace protocan_gen
