#include "generator.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <map>
#include <sstream>
#include <string>

#include <google/protobuf/compiler/code_generator.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>

// Generated from options.proto and descriptor.proto
#include "protocan/descriptor.pb.h"
#include "protocan/options.pb.h"

#include "naming.hpp"

namespace protocan_gen
{

// ─── Utilities ────────────────────────────────────────────────────────────────

static void write_string(google::protobuf::io::ZeroCopyOutputStream * os, const std::string & s)
{
  const char * data      = s.data();
  size_t       remaining = s.size();
  while (remaining > 0) {
    void * buf;
    int    size;
    if (!os->Next(&buf, &size)) break;
    size_t to_copy = std::min(remaining, static_cast<size_t>(size));
    std::memcpy(buf, data, to_copy);
    data      += to_copy;
    remaining -= to_copy;
    if (to_copy < static_cast<size_t>(size)) {
      os->BackUp(static_cast<int>(size - to_copy));
    }
  }
}

static uint32_t fnv1a_32(const uint8_t * data, size_t len)
{
  uint32_t hash = 0x811c9dc5u;
  for (size_t i = 0; i < len; ++i) {
    hash ^= data[i];
    hash *= 0x01000193u;
  }
  return hash;
}

static std::string to_hex_array(const std::string & bytes)
{
  std::string out;
  out.reserve(bytes.size() * 6);
  for (size_t i = 0; i < bytes.size(); ++i) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "0x%02X", static_cast<unsigned char>(bytes[i]));
    if (i > 0) out += ", ";
    if (i > 0 && i % 16 == 0) out += "\n  ";
    out += buf;
  }
  return out;
}

// ─── Type mapping ─────────────────────────────────────────────────────────────

struct TypeInfo
{
  std::string cpp_type;
  int         field_type_enum;  // protocan::FieldType
  int         packed_size;
};

static bool map_field_type(
  const google::protobuf::FieldDescriptor * fd,
  uint32_t                                  size_opt,
  TypeInfo &                                out,
  std::string *                             error)
{
  using FD = google::protobuf::FieldDescriptor;

  // Unsupported: repeated, map, oneof members
  if (fd->is_repeated()) {
    *error = "Field '" + fd->name() + "': repeated fields are not supported";
    return false;
  }
  if (fd->containing_oneof()) {
    *error = "Field '" + fd->name() + "': oneof fields are not supported";
    return false;
  }

  switch (fd->type()) {
    case FD::TYPE_BOOL:
      out = {"bool", protocan::FIELD_TYPE_BOOL, 1};
      return true;

    case FD::TYPE_UINT32:
      if (size_opt == 1) {
        out = {"uint8_t", protocan::FIELD_TYPE_UINT8, 1};
      } else if (size_opt == 2) {
        out = {"uint16_t", protocan::FIELD_TYPE_UINT16, 2};
      } else if (size_opt == 0 || size_opt == 4) {
        out = {"uint32_t", protocan::FIELD_TYPE_UINT32, 4};
      } else {
        *error = "Field '" + fd->name() + "': size must be 1 or 2";
        return false;
      }
      return true;

    case FD::TYPE_INT32:
      if (size_opt == 1) {
        out = {"int8_t", protocan::FIELD_TYPE_INT8, 1};
      } else if (size_opt == 2) {
        out = {"int16_t", protocan::FIELD_TYPE_INT16, 2};
      } else if (size_opt == 0 || size_opt == 4) {
        out = {"int32_t", protocan::FIELD_TYPE_INT32, 4};
      } else {
        *error = "Field '" + fd->name() + "': size must be 1 or 2";
        return false;
      }
      return true;

    case FD::TYPE_FLOAT:
      out = {"float", protocan::FIELD_TYPE_FLOAT, 4};
      return true;

    case FD::TYPE_DOUBLE:
      out = {"double", protocan::FIELD_TYPE_DOUBLE, 8};
      return true;

    case FD::TYPE_UINT64:
      out = {"uint64_t", protocan::FIELD_TYPE_UINT64, 8};
      return true;

    case FD::TYPE_INT64:
      out = {"int64_t", protocan::FIELD_TYPE_INT64, 8};
      return true;

    default:
      break;
  }

  *error = "Field '" + fd->name() + "': type '" + fd->type_name() +
           "' is not supported by protoc-gen-protocan";
  return false;
}

// ─── Message analysis ─────────────────────────────────────────────────────────

static bool analyze_message(
  const google::protobuf::Descriptor * desc,
  MessageInfo &                        out,
  std::string *                        error)
{
  if (desc->full_name() == "google.protobuf.Empty") {
    out.proto_name    = "Empty";
    out.ros2_msg_type = "";
    out.packed_size   = 0;
    out.fields        = {};
    out.is_empty      = true;
    return true;
  }

  out.proto_name = desc->name();
  out.is_empty   = false;

  // Get message-level ros2_msg_type option
  if (desc->options().HasExtension(protocan::msg)) {
    out.ros2_msg_type = desc->options().GetExtension(protocan::msg).ros2_msg_type();
  }

  // Sort fields by field number (proto3 guarantees they're in proto file order
  // but we sort explicitly for correctness)
  std::vector<const google::protobuf::FieldDescriptor *> fields;
  for (int i = 0; i < desc->field_count(); ++i) {
    fields.push_back(desc->field(i));
  }
  std::sort(fields.begin(), fields.end(), [](auto * a, auto * b) {
    return a->number() < b->number();
  });

  int offset = 0;
  for (auto * fd : fields) {
    // Get field-level options
    uint32_t    size_opt   = 0;
    std::string ros2_field = "";
    if (fd->options().HasExtension(protocan::field)) {
      const auto & fo = fd->options().GetExtension(protocan::field);
      size_opt        = fo.size();
      ros2_field      = fo.ros2_field();
    }

    TypeInfo ti;
    if (!map_field_type(fd, size_opt, ti, error)) {
      return false;
    }

    FieldInfo fi;
    fi.name            = fd->name();
    fi.cpp_type        = ti.cpp_type;
    fi.field_type_enum = ti.field_type_enum;
    fi.packed_size     = ti.packed_size;
    fi.offset          = offset;
    fi.ros2_field      = ros2_field;
    out.fields.push_back(fi);

    offset += ti.packed_size;
  }

  out.packed_size = offset;

  if (out.packed_size > 64) {
    *error = "Message '" + desc->name() + "': PACKED_SIZE=" + std::to_string(out.packed_size) +
             " exceeds CAN FD payload (64 bytes)";
    return false;
  }

  return true;
}

// ─── Service analysis ─────────────────────────────────────────────────────────

static int find_or_add_message(
  std::vector<MessageInfo> &           messages,
  std::map<std::string, int> &         msg_index_map,
  const google::protobuf::Descriptor * desc,
  std::string *                        error)
{
  const std::string & full_name = desc->full_name();
  auto                it        = msg_index_map.find(full_name);
  if (it != msg_index_map.end()) {
    return it->second;
  }

  MessageInfo mi;
  if (!analyze_message(desc, mi, error)) {
    return -2;  // error
  }

  if (mi.is_empty) {
    msg_index_map[full_name] = -1;  // Empty is represented as -1
    return -1;
  }

  int idx                  = static_cast<int>(messages.size());
  msg_index_map[full_name] = idx;
  messages.push_back(std::move(mi));
  return idx;
}

static bool analyze_service(
  const google::protobuf::FileDescriptor *  file,
  const google::protobuf::ServiceDescriptor * svc,
  ServiceInfo &                             out,
  std::string *                             error)
{
  out.proto_service_name = svc->name();
  out.snake_name         = camel_to_snake(svc->name());
  out.package_name       = file->package();

  // Service-level options
  if (svc->options().HasExtension(protocan::node)) {
    out.ros2_namespace = svc->options().GetExtension(protocan::node).ros2_namespace();
  } else {
    out.ros2_namespace = out.snake_name;
  }

  std::map<std::string, int> msg_index_map;
  int                        tx_idx  = 0;
  int                        rx_idx  = 0;
  int                        svc_idx = 0;
  int                        par_idx = 0;

  for (int i = 0; i < svc->method_count(); ++i) {
    const google::protobuf::MethodDescriptor * method = svc->method(i);

    bool client_stream = method->client_streaming();
    bool server_stream = method->server_streaming();

    if (client_stream && server_stream) {
      *error = "RPC '" + method->name() + "': bidirectional streaming is not supported";
      return false;
    }

    // Get method options
    bool        periodic      = false;
    uint32_t    priority      = 3;  // default MID
    bool        is_parameter  = false;
    bool        read_only     = false;
    std::string ros2_name;
    std::string ros2_srv_type;

    if (method->options().HasExtension(protocan::method)) {
      const auto & mo = method->options().GetExtension(protocan::method);
      periodic        = mo.periodic();
      priority        = mo.priority() > 0 ? mo.priority() : 3;
      is_parameter    = mo.is_parameter();
      read_only       = mo.read_only();
      ros2_name       = mo.ros2_name();
      ros2_srv_type   = mo.ros2_srv_type();
    }

    // Classify RPC kind
    RpcKind kind;
    if (server_stream) {
      kind = RpcKind::TX_TOPIC;
    } else if (client_stream) {
      kind = RpcKind::RX_TOPIC;
    } else if (is_parameter) {
      bool input_is_empty =
        method->input_type()->full_name() == "google.protobuf.Empty";
      kind = input_is_empty ? RpcKind::PARAM_RO : RpcKind::PARAM_RW;
    } else {
      kind = RpcKind::SERVICE;
    }

    // Resolve messages
    int req_idx = find_or_add_message(out.messages, msg_index_map, method->input_type(), error);
    if (req_idx == -2) return false;

    int res_idx = find_or_add_message(out.messages, msg_index_map, method->output_type(), error);
    if (res_idx == -2) return false;

    // Validate service response size (warning only)
    if (kind == RpcKind::SERVICE && res_idx >= 0) {
      const MessageInfo & res_msg = out.messages[static_cast<size_t>(res_idx)];
      if (res_msg.packed_size > 62) {
        // Warning (not error): could use BULK
        (void)res_msg;
      }
    }

    RpcInfo rpc;
    rpc.kind             = kind;
    rpc.proto_name       = method->name();
    rpc.snake_name       = ros2_name.empty() ? camel_to_snake(method->name()) : ros2_name;
    rpc.periodic         = periodic;
    rpc.priority         = priority;
    rpc.read_only        = read_only;
    rpc.ros2_name        = ros2_name;
    rpc.ros2_srv_type    = ros2_srv_type;
    rpc.request_msg_idx  = req_idx;
    rpc.response_msg_idx = res_idx;

    switch (kind) {
      case RpcKind::TX_TOPIC:
        rpc.index = tx_idx++;
        break;
      case RpcKind::RX_TOPIC:
        rpc.index = rx_idx++;
        break;
      case RpcKind::SERVICE:
        rpc.index = svc_idx++;
        break;
      case RpcKind::PARAM_RW:
      case RpcKind::PARAM_RO:
        rpc.index = par_idx++;
        break;
    }

    out.rpcs.push_back(std::move(rpc));
  }

  out.tx_topic_count = tx_idx;
  out.rx_topic_count = rx_idx;
  out.service_count  = svc_idx;
  out.param_count    = par_idx;

  return true;
}

// ─── Descriptor Blob ──────────────────────────────────────────────────────────

static void fill_message_descriptor(
  protocan::MessageDescriptor * md,
  const MessageInfo &           msg)
{
  md->set_ros2_msg_type(msg.ros2_msg_type);
  md->set_payload_size(static_cast<uint32_t>(msg.packed_size));
  for (const auto & f : msg.fields) {
    auto * fd = md->add_fields();
    fd->set_name(f.name);
    fd->set_type(static_cast<protocan::FieldType>(f.field_type_enum));
    fd->set_offset(static_cast<uint32_t>(f.offset));
    fd->set_size(static_cast<uint32_t>(f.packed_size));
    fd->set_ros2_field(f.ros2_field);
  }
}

static std::string build_descriptor_blob(const ServiceInfo & si)
{
  protocan::NodeDescriptor nd;
  nd.set_schema_hash(0);

  std::string node_type = si.package_name.empty()
    ? si.proto_service_name
    : si.package_name + "." + si.proto_service_name;
  nd.set_node_type_name(node_type);
  nd.set_ros2_namespace(si.ros2_namespace);

  for (const auto & rpc : si.rpcs) {
    if (rpc.kind == RpcKind::TX_TOPIC) {
      auto * td = nd.add_topics();
      td->set_index(static_cast<uint32_t>(rpc.index));
      td->set_name(rpc.ros2_name.empty() ? rpc.snake_name : rpc.ros2_name);
      td->set_is_tx(true);
      td->set_periodic(rpc.periodic);
      td->set_priority(rpc.priority);
      if (rpc.response_msg_idx >= 0) {
        fill_message_descriptor(td->mutable_message(), si.messages[rpc.response_msg_idx]);
      }
    } else if (rpc.kind == RpcKind::RX_TOPIC) {
      auto * td = nd.add_topics();
      td->set_index(static_cast<uint32_t>(rpc.index));
      td->set_name(rpc.ros2_name.empty() ? rpc.snake_name : rpc.ros2_name);
      td->set_is_tx(false);
      td->set_periodic(false);
      td->set_priority(rpc.priority);
      if (rpc.request_msg_idx >= 0) {
        fill_message_descriptor(td->mutable_message(), si.messages[rpc.request_msg_idx]);
      }
    } else if (rpc.kind == RpcKind::SERVICE) {
      auto * sd = nd.add_services();
      sd->set_index(static_cast<uint32_t>(rpc.index));
      sd->set_name(rpc.ros2_name.empty() ? rpc.snake_name : rpc.ros2_name);
      sd->set_ros2_srv_type(rpc.ros2_srv_type);
      if (rpc.request_msg_idx >= 0) {
        fill_message_descriptor(sd->mutable_request(), si.messages[rpc.request_msg_idx]);
      }
      if (rpc.response_msg_idx >= 0) {
        fill_message_descriptor(sd->mutable_response(), si.messages[rpc.response_msg_idx]);
      }
    } else {
      // PARAM_RW or PARAM_RO
      auto * pd = nd.add_params();
      pd->set_index(static_cast<uint32_t>(rpc.index));
      pd->set_name(rpc.ros2_name.empty() ? rpc.snake_name : rpc.ros2_name);
      pd->set_read_only(rpc.kind == RpcKind::PARAM_RO);

      // Determine FieldType from response message's first field (GET result)
      protocan::FieldType param_type = protocan::FIELD_TYPE_UINT8;
      int                 msg_idx    = rpc.response_msg_idx;
      if (msg_idx < 0) msg_idx = rpc.request_msg_idx;
      if (msg_idx >= 0 && !si.messages[msg_idx].fields.empty()) {
        param_type = static_cast<protocan::FieldType>(si.messages[msg_idx].fields[0].field_type_enum);
      }
      pd->set_type(param_type);
    }
  }

  // Serialize deterministically with schema_hash=0, compute FNV-1a, re-serialize
  auto deterministic_serialize = [](const protocan::NodeDescriptor & msg) -> std::string {
    std::string out;
    google::protobuf::io::StringOutputStream sos(&out);
    google::protobuf::io::CodedOutputStream cos(&sos);
    cos.SetSerializationDeterministic(true);
    msg.SerializeToCodedStream(&cos);
    return out;
  };

  std::string tmp_blob = deterministic_serialize(nd);

  uint32_t hash = fnv1a_32(
    reinterpret_cast<const uint8_t *>(tmp_blob.data()), tmp_blob.size());
  nd.set_schema_hash(hash);

  return deterministic_serialize(nd);
}

// ─── Code emission helpers ────────────────────────────────────────────────────

static std::string gen_encode_field(const FieldInfo & f)
{
  std::ostringstream os;
  const int          off = f.offset;
  const std::string & n  = f.name;

  switch (f.field_type_enum) {
    case protocan::FIELD_TYPE_BOOL:
    case protocan::FIELD_TYPE_UINT8:
      os << "  buf[" << off << "] = static_cast<uint8_t>(" << n << ");\n";
      break;
    case protocan::FIELD_TYPE_INT8:
      os << "  buf[" << off << "] = static_cast<uint8_t>(" << n << ");\n";
      break;
    case protocan::FIELD_TYPE_UINT16:
      os << "  buf[" << off << "] = static_cast<uint8_t>(" << n << ");\n";
      os << "  buf[" << off + 1 << "] = static_cast<uint8_t>((" << n << ") >> 8);\n";
      break;
    case protocan::FIELD_TYPE_INT16:
      os << "  { uint16_t _v = static_cast<uint16_t>(" << n << ");\n";
      os << "    buf[" << off << "] = static_cast<uint8_t>(_v);\n";
      os << "    buf[" << off + 1 << "] = static_cast<uint8_t>(_v >> 8); }\n";
      break;
    case protocan::FIELD_TYPE_UINT32:
      os << "  buf[" << off << "] = static_cast<uint8_t>(" << n << ");\n";
      os << "  buf[" << off + 1 << "] = static_cast<uint8_t>((" << n << ") >> 8);\n";
      os << "  buf[" << off + 2 << "] = static_cast<uint8_t>((" << n << ") >> 16);\n";
      os << "  buf[" << off + 3 << "] = static_cast<uint8_t>((" << n << ") >> 24);\n";
      break;
    case protocan::FIELD_TYPE_INT32:
      os << "  { uint32_t _v = static_cast<uint32_t>(" << n << ");\n";
      os << "    buf[" << off << "] = static_cast<uint8_t>(_v);\n";
      os << "    buf[" << off + 1 << "] = static_cast<uint8_t>(_v >> 8);\n";
      os << "    buf[" << off + 2 << "] = static_cast<uint8_t>(_v >> 16);\n";
      os << "    buf[" << off + 3 << "] = static_cast<uint8_t>(_v >> 24); }\n";
      break;
    case protocan::FIELD_TYPE_FLOAT:
      os << "  { uint32_t _v; std::memcpy(&_v, &" << n << ", 4);\n";
      os << "    buf[" << off << "] = static_cast<uint8_t>(_v);\n";
      os << "    buf[" << off + 1 << "] = static_cast<uint8_t>(_v >> 8);\n";
      os << "    buf[" << off + 2 << "] = static_cast<uint8_t>(_v >> 16);\n";
      os << "    buf[" << off + 3 << "] = static_cast<uint8_t>(_v >> 24); }\n";
      break;
    case protocan::FIELD_TYPE_DOUBLE:
      os << "  { uint64_t _v; std::memcpy(&_v, &" << n << ", 8);\n";
      os << "    buf[" << off << "] = static_cast<uint8_t>(_v);\n";
      os << "    buf[" << off + 1 << "] = static_cast<uint8_t>(_v >> 8);\n";
      os << "    buf[" << off + 2 << "] = static_cast<uint8_t>(_v >> 16);\n";
      os << "    buf[" << off + 3 << "] = static_cast<uint8_t>(_v >> 24);\n";
      os << "    buf[" << off + 4 << "] = static_cast<uint8_t>(_v >> 32);\n";
      os << "    buf[" << off + 5 << "] = static_cast<uint8_t>(_v >> 40);\n";
      os << "    buf[" << off + 6 << "] = static_cast<uint8_t>(_v >> 48);\n";
      os << "    buf[" << off + 7 << "] = static_cast<uint8_t>(_v >> 56); }\n";
      break;
    case protocan::FIELD_TYPE_UINT64:
      os << "  buf[" << off << "] = static_cast<uint8_t>(" << n << ");\n";
      os << "  buf[" << off + 1 << "] = static_cast<uint8_t>((" << n << ") >> 8);\n";
      os << "  buf[" << off + 2 << "] = static_cast<uint8_t>((" << n << ") >> 16);\n";
      os << "  buf[" << off + 3 << "] = static_cast<uint8_t>((" << n << ") >> 24);\n";
      os << "  buf[" << off + 4 << "] = static_cast<uint8_t>((" << n << ") >> 32);\n";
      os << "  buf[" << off + 5 << "] = static_cast<uint8_t>((" << n << ") >> 40);\n";
      os << "  buf[" << off + 6 << "] = static_cast<uint8_t>((" << n << ") >> 48);\n";
      os << "  buf[" << off + 7 << "] = static_cast<uint8_t>((" << n << ") >> 56);\n";
      break;
    case protocan::FIELD_TYPE_INT64:
      os << "  { uint64_t _v = static_cast<uint64_t>(" << n << ");\n";
      os << "    buf[" << off << "] = static_cast<uint8_t>(_v);\n";
      os << "    buf[" << off + 1 << "] = static_cast<uint8_t>(_v >> 8);\n";
      os << "    buf[" << off + 2 << "] = static_cast<uint8_t>(_v >> 16);\n";
      os << "    buf[" << off + 3 << "] = static_cast<uint8_t>(_v >> 24);\n";
      os << "    buf[" << off + 4 << "] = static_cast<uint8_t>(_v >> 32);\n";
      os << "    buf[" << off + 5 << "] = static_cast<uint8_t>(_v >> 40);\n";
      os << "    buf[" << off + 6 << "] = static_cast<uint8_t>(_v >> 48);\n";
      os << "    buf[" << off + 7 << "] = static_cast<uint8_t>(_v >> 56); }\n";
      break;
  }
  return os.str();
}

static std::string gen_decode_field(const FieldInfo & f, const std::string & msg_var)
{
  std::ostringstream os;
  const int          off = f.offset;
  const std::string  lhs = msg_var + "." + f.name;

  switch (f.field_type_enum) {
    case protocan::FIELD_TYPE_BOOL:
      os << "  " << lhs << " = (buf[" << off << "] != 0);\n";
      break;
    case protocan::FIELD_TYPE_UINT8:
      os << "  " << lhs << " = buf[" << off << "];\n";
      break;
    case protocan::FIELD_TYPE_INT8:
      os << "  " << lhs << " = static_cast<int8_t>(buf[" << off << "]);\n";
      break;
    case protocan::FIELD_TYPE_UINT16:
      os << "  " << lhs << " = static_cast<uint16_t>(buf[" << off << "])"
         << " | (static_cast<uint16_t>(buf[" << off + 1 << "]) << 8);\n";
      break;
    case protocan::FIELD_TYPE_INT16:
      os << "  { uint16_t _v = static_cast<uint16_t>(buf[" << off << "])"
         << " | (static_cast<uint16_t>(buf[" << off + 1 << "]) << 8);\n";
      os << "    " << lhs << " = static_cast<int16_t>(_v); }\n";
      break;
    case protocan::FIELD_TYPE_UINT32:
      os << "  " << lhs << " = static_cast<uint32_t>(buf[" << off << "])"
         << " | (static_cast<uint32_t>(buf[" << off + 1 << "]) << 8)"
         << " | (static_cast<uint32_t>(buf[" << off + 2 << "]) << 16)"
         << " | (static_cast<uint32_t>(buf[" << off + 3 << "]) << 24);\n";
      break;
    case protocan::FIELD_TYPE_INT32:
      os << "  { uint32_t _v = static_cast<uint32_t>(buf[" << off << "])"
         << " | (static_cast<uint32_t>(buf[" << off + 1 << "]) << 8)"
         << " | (static_cast<uint32_t>(buf[" << off + 2 << "]) << 16)"
         << " | (static_cast<uint32_t>(buf[" << off + 3 << "]) << 24);\n";
      os << "    " << lhs << " = static_cast<int32_t>(_v); }\n";
      break;
    case protocan::FIELD_TYPE_FLOAT:
      os << "  { uint32_t _v = static_cast<uint32_t>(buf[" << off << "])"
         << " | (static_cast<uint32_t>(buf[" << off + 1 << "]) << 8)"
         << " | (static_cast<uint32_t>(buf[" << off + 2 << "]) << 16)"
         << " | (static_cast<uint32_t>(buf[" << off + 3 << "]) << 24);\n";
      os << "    std::memcpy(&" << lhs << ", &_v, 4); }\n";
      break;
    case protocan::FIELD_TYPE_DOUBLE:
      os << "  { uint64_t _v = static_cast<uint64_t>(buf[" << off << "])"
         << " | (static_cast<uint64_t>(buf[" << off + 1 << "]) << 8)"
         << " | (static_cast<uint64_t>(buf[" << off + 2 << "]) << 16)"
         << " | (static_cast<uint64_t>(buf[" << off + 3 << "]) << 24)"
         << " | (static_cast<uint64_t>(buf[" << off + 4 << "]) << 32)"
         << " | (static_cast<uint64_t>(buf[" << off + 5 << "]) << 40)"
         << " | (static_cast<uint64_t>(buf[" << off + 6 << "]) << 48)"
         << " | (static_cast<uint64_t>(buf[" << off + 7 << "]) << 56);\n";
      os << "    std::memcpy(&" << lhs << ", &_v, 8); }\n";
      break;
    case protocan::FIELD_TYPE_UINT64:
      os << "  " << lhs << " = static_cast<uint64_t>(buf[" << off << "])"
         << " | (static_cast<uint64_t>(buf[" << off + 1 << "]) << 8)"
         << " | (static_cast<uint64_t>(buf[" << off + 2 << "]) << 16)"
         << " | (static_cast<uint64_t>(buf[" << off + 3 << "]) << 24)"
         << " | (static_cast<uint64_t>(buf[" << off + 4 << "]) << 32)"
         << " | (static_cast<uint64_t>(buf[" << off + 5 << "]) << 40)"
         << " | (static_cast<uint64_t>(buf[" << off + 6 << "]) << 48)"
         << " | (static_cast<uint64_t>(buf[" << off + 7 << "]) << 56);\n";
      break;
    case protocan::FIELD_TYPE_INT64:
      os << "  { uint64_t _v = static_cast<uint64_t>(buf[" << off << "])"
         << " | (static_cast<uint64_t>(buf[" << off + 1 << "]) << 8)"
         << " | (static_cast<uint64_t>(buf[" << off + 2 << "]) << 16)"
         << " | (static_cast<uint64_t>(buf[" << off + 3 << "]) << 24)"
         << " | (static_cast<uint64_t>(buf[" << off + 4 << "]) << 32)"
         << " | (static_cast<uint64_t>(buf[" << off + 5 << "]) << 40)"
         << " | (static_cast<uint64_t>(buf[" << off + 6 << "]) << 48)"
         << " | (static_cast<uint64_t>(buf[" << off + 7 << "]) << 56);\n";
      os << "    " << lhs << " = static_cast<int64_t>(_v); }\n";
      break;
  }
  return os.str();
}

// eoffset variants: use e.offset as the runtime base instead of compile-time f.offset

static std::string gen_encode_field_to_eoffset(const FieldInfo & f, const std::string & src)
{
  std::ostringstream os;
  const std::string & n = src;

  switch (f.field_type_enum) {
    case protocan::FIELD_TYPE_BOOL:
    case protocan::FIELD_TYPE_UINT8:
      os << "  buf[e.offset + 0] = static_cast<uint8_t>(" << n << ");\n";
      break;
    case protocan::FIELD_TYPE_INT8:
      os << "  buf[e.offset + 0] = static_cast<uint8_t>(" << n << ");\n";
      break;
    case protocan::FIELD_TYPE_UINT16:
      os << "  buf[e.offset + 0] = static_cast<uint8_t>(" << n << ");\n";
      os << "  buf[e.offset + 1] = static_cast<uint8_t>((" << n << ") >> 8);\n";
      break;
    case protocan::FIELD_TYPE_INT16:
      os << "  { uint16_t _v = static_cast<uint16_t>(" << n << ");\n";
      os << "    buf[e.offset + 0] = static_cast<uint8_t>(_v);\n";
      os << "    buf[e.offset + 1] = static_cast<uint8_t>(_v >> 8); }\n";
      break;
    case protocan::FIELD_TYPE_UINT32:
      os << "  buf[e.offset + 0] = static_cast<uint8_t>(" << n << ");\n";
      os << "  buf[e.offset + 1] = static_cast<uint8_t>((" << n << ") >> 8);\n";
      os << "  buf[e.offset + 2] = static_cast<uint8_t>((" << n << ") >> 16);\n";
      os << "  buf[e.offset + 3] = static_cast<uint8_t>((" << n << ") >> 24);\n";
      break;
    case protocan::FIELD_TYPE_INT32:
      os << "  { uint32_t _v = static_cast<uint32_t>(" << n << ");\n";
      os << "    buf[e.offset + 0] = static_cast<uint8_t>(_v);\n";
      os << "    buf[e.offset + 1] = static_cast<uint8_t>(_v >> 8);\n";
      os << "    buf[e.offset + 2] = static_cast<uint8_t>(_v >> 16);\n";
      os << "    buf[e.offset + 3] = static_cast<uint8_t>(_v >> 24); }\n";
      break;
    case protocan::FIELD_TYPE_FLOAT:
      os << "  { uint32_t _v; std::memcpy(&_v, &" << n << ", 4);\n";
      os << "    buf[e.offset + 0] = static_cast<uint8_t>(_v);\n";
      os << "    buf[e.offset + 1] = static_cast<uint8_t>(_v >> 8);\n";
      os << "    buf[e.offset + 2] = static_cast<uint8_t>(_v >> 16);\n";
      os << "    buf[e.offset + 3] = static_cast<uint8_t>(_v >> 24); }\n";
      break;
    case protocan::FIELD_TYPE_DOUBLE:
      os << "  { uint64_t _v; std::memcpy(&_v, &" << n << ", 8);\n";
      os << "    buf[e.offset + 0] = static_cast<uint8_t>(_v);\n";
      os << "    buf[e.offset + 1] = static_cast<uint8_t>(_v >> 8);\n";
      os << "    buf[e.offset + 2] = static_cast<uint8_t>(_v >> 16);\n";
      os << "    buf[e.offset + 3] = static_cast<uint8_t>(_v >> 24);\n";
      os << "    buf[e.offset + 4] = static_cast<uint8_t>(_v >> 32);\n";
      os << "    buf[e.offset + 5] = static_cast<uint8_t>(_v >> 40);\n";
      os << "    buf[e.offset + 6] = static_cast<uint8_t>(_v >> 48);\n";
      os << "    buf[e.offset + 7] = static_cast<uint8_t>(_v >> 56); }\n";
      break;
    case protocan::FIELD_TYPE_UINT64:
      os << "  buf[e.offset + 0] = static_cast<uint8_t>(" << n << ");\n";
      os << "  buf[e.offset + 1] = static_cast<uint8_t>((" << n << ") >> 8);\n";
      os << "  buf[e.offset + 2] = static_cast<uint8_t>((" << n << ") >> 16);\n";
      os << "  buf[e.offset + 3] = static_cast<uint8_t>((" << n << ") >> 24);\n";
      os << "  buf[e.offset + 4] = static_cast<uint8_t>((" << n << ") >> 32);\n";
      os << "  buf[e.offset + 5] = static_cast<uint8_t>((" << n << ") >> 40);\n";
      os << "  buf[e.offset + 6] = static_cast<uint8_t>((" << n << ") >> 48);\n";
      os << "  buf[e.offset + 7] = static_cast<uint8_t>((" << n << ") >> 56);\n";
      break;
    case protocan::FIELD_TYPE_INT64:
      os << "  { uint64_t _v = static_cast<uint64_t>(" << n << ");\n";
      os << "    buf[e.offset + 0] = static_cast<uint8_t>(_v);\n";
      os << "    buf[e.offset + 1] = static_cast<uint8_t>(_v >> 8);\n";
      os << "    buf[e.offset + 2] = static_cast<uint8_t>(_v >> 16);\n";
      os << "    buf[e.offset + 3] = static_cast<uint8_t>(_v >> 24);\n";
      os << "    buf[e.offset + 4] = static_cast<uint8_t>(_v >> 32);\n";
      os << "    buf[e.offset + 5] = static_cast<uint8_t>(_v >> 40);\n";
      os << "    buf[e.offset + 6] = static_cast<uint8_t>(_v >> 48);\n";
      os << "    buf[e.offset + 7] = static_cast<uint8_t>(_v >> 56); }\n";
      break;
  }
  return os.str();
}

static std::string gen_decode_field_from_eoffset(const FieldInfo & f, const std::string & lhs)
{
  std::ostringstream os;

  switch (f.field_type_enum) {
    case protocan::FIELD_TYPE_BOOL:
      os << "  " << lhs << " = (data[e.offset + 0] != 0);\n";
      break;
    case protocan::FIELD_TYPE_UINT8:
      os << "  " << lhs << " = data[e.offset + 0];\n";
      break;
    case protocan::FIELD_TYPE_INT8:
      os << "  " << lhs << " = static_cast<int8_t>(data[e.offset + 0]);\n";
      break;
    case protocan::FIELD_TYPE_UINT16:
      os << "  " << lhs << " = static_cast<uint16_t>(data[e.offset + 0])"
         << " | (static_cast<uint16_t>(data[e.offset + 1]) << 8);\n";
      break;
    case protocan::FIELD_TYPE_INT16:
      os << "  { uint16_t _v = static_cast<uint16_t>(data[e.offset + 0])"
         << " | (static_cast<uint16_t>(data[e.offset + 1]) << 8);\n";
      os << "    " << lhs << " = static_cast<int16_t>(_v); }\n";
      break;
    case protocan::FIELD_TYPE_UINT32:
      os << "  " << lhs << " = static_cast<uint32_t>(data[e.offset + 0])"
         << " | (static_cast<uint32_t>(data[e.offset + 1]) << 8)"
         << " | (static_cast<uint32_t>(data[e.offset + 2]) << 16)"
         << " | (static_cast<uint32_t>(data[e.offset + 3]) << 24);\n";
      break;
    case protocan::FIELD_TYPE_INT32:
      os << "  { uint32_t _v = static_cast<uint32_t>(data[e.offset + 0])"
         << " | (static_cast<uint32_t>(data[e.offset + 1]) << 8)"
         << " | (static_cast<uint32_t>(data[e.offset + 2]) << 16)"
         << " | (static_cast<uint32_t>(data[e.offset + 3]) << 24);\n";
      os << "    " << lhs << " = static_cast<int32_t>(_v); }\n";
      break;
    case protocan::FIELD_TYPE_FLOAT:
      os << "  { uint32_t _v = static_cast<uint32_t>(data[e.offset + 0])"
         << " | (static_cast<uint32_t>(data[e.offset + 1]) << 8)"
         << " | (static_cast<uint32_t>(data[e.offset + 2]) << 16)"
         << " | (static_cast<uint32_t>(data[e.offset + 3]) << 24);\n";
      os << "    std::memcpy(&" << lhs << ", &_v, 4); }\n";
      break;
    case protocan::FIELD_TYPE_DOUBLE:
      os << "  { uint64_t _v = static_cast<uint64_t>(data[e.offset + 0])"
         << " | (static_cast<uint64_t>(data[e.offset + 1]) << 8)"
         << " | (static_cast<uint64_t>(data[e.offset + 2]) << 16)"
         << " | (static_cast<uint64_t>(data[e.offset + 3]) << 24)"
         << " | (static_cast<uint64_t>(data[e.offset + 4]) << 32)"
         << " | (static_cast<uint64_t>(data[e.offset + 5]) << 40)"
         << " | (static_cast<uint64_t>(data[e.offset + 6]) << 48)"
         << " | (static_cast<uint64_t>(data[e.offset + 7]) << 56);\n";
      os << "    std::memcpy(&" << lhs << ", &_v, 8); }\n";
      break;
    case protocan::FIELD_TYPE_UINT64:
      os << "  " << lhs << " = static_cast<uint64_t>(data[e.offset + 0])"
         << " | (static_cast<uint64_t>(data[e.offset + 1]) << 8)"
         << " | (static_cast<uint64_t>(data[e.offset + 2]) << 16)"
         << " | (static_cast<uint64_t>(data[e.offset + 3]) << 24)"
         << " | (static_cast<uint64_t>(data[e.offset + 4]) << 32)"
         << " | (static_cast<uint64_t>(data[e.offset + 5]) << 40)"
         << " | (static_cast<uint64_t>(data[e.offset + 6]) << 48)"
         << " | (static_cast<uint64_t>(data[e.offset + 7]) << 56);\n";
      break;
    case protocan::FIELD_TYPE_INT64:
      os << "  { uint64_t _v = static_cast<uint64_t>(data[e.offset + 0])"
         << " | (static_cast<uint64_t>(data[e.offset + 1]) << 8)"
         << " | (static_cast<uint64_t>(data[e.offset + 2]) << 16)"
         << " | (static_cast<uint64_t>(data[e.offset + 3]) << 24)"
         << " | (static_cast<uint64_t>(data[e.offset + 4]) << 32)"
         << " | (static_cast<uint64_t>(data[e.offset + 5]) << 40)"
         << " | (static_cast<uint64_t>(data[e.offset + 6]) << 48)"
         << " | (static_cast<uint64_t>(data[e.offset + 7]) << 56);\n";
      os << "    " << lhs << " = static_cast<int64_t>(_v); }\n";
      break;
  }
  return os.str();
}

// ─── HPP generation ───────────────────────────────────────────────────────────

static std::string ns_open(const std::string & package)
{
  return package.empty() ? "namespace protocan {\n" : "namespace protocan::" + package + " {\n";
}

static std::string ns_close(const std::string & package)
{
  return package.empty() ? "}  // namespace protocan\n"
                         : "}  // namespace protocan::" + package + "\n";
}

static std::string gen_hpp(const ServiceInfo & si)
{
  std::ostringstream o;

  o << "#pragma once\n";
  o << "#include <cstddef>\n";
  o << "#include <cstdint>\n";
  o << "#include <cstring>\n";
  o << "#include \"protocan_device/node_base.hpp\"\n";
  o << "\n";
  o << ns_open(si.package_name);
  o << "\n";

  // Schema constants
  // (SCHEMA_HASH is computed at generation time and emitted in .cpp;
  //  here we forward-declare it via extern and provide a constexpr alias
  //  computed from the blob. We emit it in cpp and re-export here as extern.)
  // Actually: SCHEMA_HASH is constexpr so it must be in the header.
  // We will emit the actual value in the cpp and also in the header via
  // a constexpr that matches. We store it as a placeholder here and fill it in.
  // NOTE: We emit the real value; the blob is built in gen_cpp().
  // For the header, we need to know SCHEMA_HASH at header-gen time too.
  // We compute it once, and pass it along. Since gen_hpp and gen_cpp share
  // the same ServiceInfo, we store the hash in ServiceInfo... but we don't.
  // Solution: compute blob early, then store hash in ServiceInfo. But ServiceInfo
  // is const here. We'll compute hash in Generate() and store in a local.
  // For now, emit a placeholder comment and fill via a macro trick.
  // Actually the cleanest solution: compute blob in Generate(), store hash value
  // in ServiceInfo or a local struct, then pass to both gen_hpp and gen_cpp.
  // We'll add schema_hash to ServiceInfo after analysis.
  // Since we can't modify ServiceInfo (const &), use a wrapper struct.
  // SIMPLEST: just emit the hash value directly (caller computes it first).
  // We'll emit it as SCHEMA_HASH_PLACEHOLDER and the caller replaces it.
  // But that's ugly. Let's just compute the blob in a side channel.
  // The gen_hpp function will compute the blob itself (same as gen_cpp).
  // Since deterministic serialization gives the same result, this is fine.

  std::string blob = build_descriptor_blob(si);
  protocan::NodeDescriptor nd_check;
  nd_check.ParseFromString(blob);
  uint32_t schema_hash = nd_check.schema_hash();

  char hash_buf[16];
  std::snprintf(hash_buf, sizeof(hash_buf), "0x%08Xu", schema_hash);

  // PDO エントリ数 = 全 TX/RX トピックのフィールド数の合計
  int tx_pdo_field_count = 0;
  int rx_pdo_field_count = 0;
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind == RpcKind::TX_TOPIC && rpc.response_msg_idx >= 0) {
      tx_pdo_field_count +=
        static_cast<int>(si.messages[rpc.response_msg_idx].fields.size());
    } else if (rpc.kind == RpcKind::RX_TOPIC && rpc.request_msg_idx >= 0) {
      rx_pdo_field_count +=
        static_cast<int>(si.messages[rpc.request_msg_idx].fields.size());
    }
  }

  o << "// ─── Schema constants ───\n";
  o << "constexpr uint32_t SCHEMA_HASH        = " << hash_buf << ";\n";
  o << "constexpr uint8_t  MAX_PDO_TX_ENTRIES = " << tx_pdo_field_count << ";\n";
  o << "constexpr uint8_t  MAX_PDO_RX_ENTRIES = " << rx_pdo_field_count << ";\n";
  o << "extern const uint8_t DESCRIPTOR_BLOB[];\n";
  o << "extern const size_t  DESCRIPTOR_BLOB_SIZE;\n";
  o << "\n";

  // Message structs (skip Empty and param-only messages)
  // Determine which message indices are used by topics and services
  std::vector<bool> need_struct(si.messages.size(), false);
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind == RpcKind::TX_TOPIC) {
      if (rpc.response_msg_idx >= 0)
        need_struct[rpc.response_msg_idx] = true;
    } else if (rpc.kind == RpcKind::RX_TOPIC) {
      if (rpc.request_msg_idx >= 0)
        need_struct[rpc.request_msg_idx] = true;
    } else if (rpc.kind == RpcKind::SERVICE) {
      if (rpc.request_msg_idx >= 0)
        need_struct[rpc.request_msg_idx] = true;
      if (rpc.response_msg_idx >= 0)
        need_struct[rpc.response_msg_idx] = true;
    }
  }

  for (size_t mi = 0; mi < si.messages.size(); ++mi) {
    if (!need_struct[mi]) continue;
    const MessageInfo & msg = si.messages[mi];

    o << "struct " << msg.proto_name << " {\n";
    for (const auto & f : msg.fields) {
      o << "  " << f.cpp_type << " " << f.name << ";\n";
    }
    o << "\n";
    o << "  static constexpr size_t PACKED_SIZE = " << msg.packed_size << ";\n";
    o << "  void encode(uint8_t * buf) const;\n";
    o << "  static " << msg.proto_name << " decode(const uint8_t * buf);\n";
    o << "};\n\n";
  }

  // Node class
  o << "class Node : public protocan::device::NodeBase {\n";
  o << "public:\n";
  o << "  Node(uint8_t local_id, const char * instance_name);\n";
  o << "  Node(const Node &) = delete;\n";
  o << "  Node & operator=(const Node &) = delete;\n";
  o << "\n";

  // TX Topics
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind != RpcKind::TX_TOPIC) continue;
    if (rpc.response_msg_idx < 0) continue;
    const std::string & msg_name = si.messages[rpc.response_msg_idx].proto_name;
    o << "  // TX Topic: " << rpc.proto_name << "\n";
    o << "  " << msg_name << " & " << rpc.snake_name << "_buffer();\n";
    o << "  protocan::Status publish_" << rpc.snake_name << "();\n\n";
  }

  // RX Topics
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind != RpcKind::RX_TOPIC) continue;
    if (rpc.request_msg_idx < 0) continue;
    const std::string & msg_name = si.messages[rpc.request_msg_idx].proto_name;
    o << "  // RX Topic: " << rpc.proto_name << "\n";
    o << "  using " << rpc.proto_name << "Callback = void(*)(const " << msg_name
      << " &, void *);\n";
    o << "  void on_" << rpc.snake_name << "(" << rpc.proto_name
      << "Callback cb, void * ctx = nullptr);\n\n";
  }

  // Services
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind != RpcKind::SERVICE) continue;
    std::string req_name = (rpc.request_msg_idx >= 0)
      ? si.messages[rpc.request_msg_idx].proto_name
      : "void";
    std::string res_name = (rpc.response_msg_idx >= 0)
      ? si.messages[rpc.response_msg_idx].proto_name
      : "void";
    o << "  // Service: " << rpc.proto_name << "\n";
    o << "  using " << rpc.proto_name << "Callback = protocan::Status(*)(const " << req_name
      << " &, " << res_name << " &, void *);\n";
    o << "  void on_" << rpc.snake_name << "(" << rpc.proto_name
      << "Callback cb, void * ctx = nullptr);\n\n";
  }

  // Parameters
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind != RpcKind::PARAM_RW && rpc.kind != RpcKind::PARAM_RO) continue;
    o << "  // Parameter: " << rpc.proto_name << "\n";
    o << "  using " << rpc.proto_name
      << "GetCallback = protocan::Status(*)(uint8_t * out, uint8_t & size, void *);\n";
    o << "  void on_get_" << rpc.snake_name << "(" << rpc.proto_name
      << "GetCallback cb, void * ctx = nullptr);\n";
    if (rpc.kind == RpcKind::PARAM_RW) {
      o << "  using " << rpc.proto_name
        << "SetCallback = protocan::Status(*)(const uint8_t * data, uint8_t size, void *);\n";
      o << "  void on_set_" << rpc.snake_name << "(" << rpc.proto_name
        << "SetCallback cb, void * ctx = nullptr);\n";
    }
    o << "\n";
  }

  // NodeBase overrides
  o << "  // NodeBase overrides\n";
  o << "  void on_pdo_rx(uint16_t pdo_id, const uint8_t * data, uint8_t len) override;\n";
  o << "  uint8_t fill_pdo_tx(uint16_t pdo_id, uint8_t * buf, uint8_t max_len) override;\n";
  o << "  protocan::Status on_param_read(uint8_t idx, uint8_t * out,"
       " uint8_t & out_size) override;\n";
  o << "  protocan::Status on_param_write(uint8_t idx, const uint8_t * data,"
       " uint8_t sz) override;\n";
  o << "  protocan::Status on_service_req(uint8_t svc, const uint8_t * req,"
       " uint8_t rsz, uint8_t * res, uint8_t & osz) override;\n";
  o << "\n";

  o << "private:\n";

  // TX buffers
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind != RpcKind::TX_TOPIC) continue;
    if (rpc.response_msg_idx < 0) continue;
    const std::string & msg_name = si.messages[rpc.response_msg_idx].proto_name;
    o << "  " << msg_name << " " << rpc.snake_name << "_buf_{};\n";
  }

  // RX buffers
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind != RpcKind::RX_TOPIC) continue;
    if (rpc.request_msg_idx < 0) continue;
    const std::string & msg_name = si.messages[rpc.request_msg_idx].proto_name;
    o << "  " << msg_name << " " << rpc.snake_name << "_rx_buf_{};\n";
  }
  o << "\n";

  // Callbacks
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind == RpcKind::TX_TOPIC) continue;  // TX has no callback
    if (rpc.kind == RpcKind::RX_TOPIC) {
      o << "  " << rpc.proto_name << "Callback " << rpc.snake_name
        << "_cb_ = nullptr; void * " << rpc.snake_name << "_ctx_ = nullptr;\n";
    } else if (rpc.kind == RpcKind::SERVICE) {
      o << "  " << rpc.proto_name << "Callback " << rpc.snake_name
        << "_cb_ = nullptr; void * " << rpc.snake_name << "_ctx_ = nullptr;\n";
    } else {
      o << "  " << rpc.proto_name << "GetCallback " << rpc.snake_name
        << "_get_cb_ = nullptr; void * " << rpc.snake_name << "_get_ctx_ = nullptr;\n";
      if (rpc.kind == RpcKind::PARAM_RW) {
        o << "  " << rpc.proto_name << "SetCallback " << rpc.snake_name
          << "_set_cb_ = nullptr; void * " << rpc.snake_name << "_set_ctx_ = nullptr;\n";
      }
    }
  }

  o << "\n";
  o << "};\n\n";

  o << ns_close(si.package_name);
  return o.str();
}

// ─── CPP generation ───────────────────────────────────────────────────────────

static std::string gen_cpp(const ServiceInfo & si)
{
  std::ostringstream o;

  // Determine include name: the .hpp is named after snake_name of service
  o << "#include \"" << si.snake_name << ".hpp\"\n";
  o << "#include <cstring>\n";
  o << "\n";
  o << ns_open(si.package_name);
  o << "\n";

  // Descriptor blob
  std::string blob = build_descriptor_blob(si);
  o << "const uint8_t DESCRIPTOR_BLOB[] = {\n";
  o << "  " << to_hex_array(blob) << "\n};\n";
  o << "const size_t DESCRIPTOR_BLOB_SIZE = sizeof(DESCRIPTOR_BLOB);\n\n";

  // Determine which messages need struct implementations
  std::vector<bool> need_struct(si.messages.size(), false);
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind == RpcKind::TX_TOPIC) {
      if (rpc.response_msg_idx >= 0) need_struct[rpc.response_msg_idx] = true;
    } else if (rpc.kind == RpcKind::RX_TOPIC) {
      if (rpc.request_msg_idx >= 0) need_struct[rpc.request_msg_idx] = true;
    } else if (rpc.kind == RpcKind::SERVICE) {
      if (rpc.request_msg_idx >= 0) need_struct[rpc.request_msg_idx] = true;
      if (rpc.response_msg_idx >= 0) need_struct[rpc.response_msg_idx] = true;
    }
  }

  // encode/decode implementations
  for (size_t mi = 0; mi < si.messages.size(); ++mi) {
    if (!need_struct[mi]) continue;
    const MessageInfo & msg = si.messages[mi];

    // encode
    o << "void " << msg.proto_name << "::encode(uint8_t * buf) const {\n";
    for (const auto & f : msg.fields) {
      o << gen_encode_field(f);
    }
    o << "}\n\n";

    // decode
    o << msg.proto_name << " " << msg.proto_name << "::decode(const uint8_t * buf) {\n";
    o << "  " << msg.proto_name << " m{};\n";
    for (const auto & f : msg.fields) {
      o << gen_decode_field(f, "m");
    }
    o << "  return m;\n";
    o << "}\n\n";
  }

  // Node constructor
  o << "Node::Node(uint8_t local_id, const char * instance_name)\n";
  o << ": protocan::device::NodeBase(local_id, instance_name, DESCRIPTOR_BLOB,"
       " DESCRIPTOR_BLOB_SIZE, SCHEMA_HASH)\n";
  o << "{}\n\n";

  // TX Topic methods
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind != RpcKind::TX_TOPIC) continue;
    if (rpc.response_msg_idx < 0) continue;
    const MessageInfo & msg = si.messages[rpc.response_msg_idx];

    // buffer()
    o << msg.proto_name << " & Node::" << rpc.snake_name << "_buffer() {\n";
    o << "  return " << rpc.snake_name << "_buf_;\n";
    o << "}\n\n";

    // publish_()
    o << "protocan::Status Node::publish_" << rpc.snake_name << "() {\n";
    o << "  bool found = false;\n";
    o << "  for (uint8_t i = 0; i < pdo_tx_count(); ++i) {\n";
    o << "    if (pdo_tx_at(i).topic_index == " << rpc.index << ") {\n";
    o << "      found = true;\n";
    o << "      request_pdo_tx(pdo_tx_at(i).pdo_id);\n";
    o << "    }\n";
    o << "  }\n";
    o << "  return found ? protocan::Status::OK : protocan::Status::NOT_FOUND;\n";
    o << "}\n\n";
  }

  // RX Topic on_ methods
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind != RpcKind::RX_TOPIC) continue;
    o << "void Node::on_" << rpc.snake_name << "(" << rpc.proto_name
      << "Callback cb, void * ctx) {\n";
    o << "  " << rpc.snake_name << "_cb_  = cb;\n";
    o << "  " << rpc.snake_name << "_ctx_ = ctx;\n";
    o << "}\n\n";
  }

  // Service on_ methods
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind != RpcKind::SERVICE) continue;
    o << "void Node::on_" << rpc.snake_name << "(" << rpc.proto_name
      << "Callback cb, void * ctx) {\n";
    o << "  " << rpc.snake_name << "_cb_  = cb;\n";
    o << "  " << rpc.snake_name << "_ctx_ = ctx;\n";
    o << "}\n\n";
  }

  // Param on_get / on_set methods
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind != RpcKind::PARAM_RW && rpc.kind != RpcKind::PARAM_RO) continue;
    o << "void Node::on_get_" << rpc.snake_name << "(" << rpc.proto_name
      << "GetCallback cb, void * ctx) {\n";
    o << "  " << rpc.snake_name << "_get_cb_  = cb;\n";
    o << "  " << rpc.snake_name << "_get_ctx_ = ctx;\n";
    o << "}\n\n";
    if (rpc.kind == RpcKind::PARAM_RW) {
      o << "void Node::on_set_" << rpc.snake_name << "(" << rpc.proto_name
        << "SetCallback cb, void * ctx) {\n";
      o << "  " << rpc.snake_name << "_set_cb_  = cb;\n";
      o << "  " << rpc.snake_name << "_set_ctx_ = ctx;\n";
      o << "}\n\n";
    }
  }

  // on_pdo_rx
  o << "void Node::on_pdo_rx(uint16_t pdo_id, const uint8_t * data, uint8_t len) {\n";
  if (si.rx_topic_count > 0) {
    // updated フラグを topic ごとに宣言
    for (const auto & rpc : si.rpcs) {
      if (rpc.kind != RpcKind::RX_TOPIC) continue;
      o << "  bool " << rpc.snake_name << "_updated = false;\n";
    }
  }
  o << "  for (uint8_t i = 0; i < pdo_rx_count(); ++i) {\n";
  o << "    const auto & e = pdo_rx_at(i);\n";
  o << "    if (e.pdo_id != pdo_id) continue;\n";
  o << "    if (e.offset + e.size > len) continue;\n";
  if (si.rx_topic_count > 0) {
    o << "    switch (e.topic_index) {\n";
    for (const auto & rpc : si.rpcs) {
      if (rpc.kind != RpcKind::RX_TOPIC) continue;
      if (rpc.request_msg_idx < 0) continue;
      const MessageInfo & msg = si.messages[rpc.request_msg_idx];
      o << "      case " << rpc.index << ": {  // " << rpc.snake_name << "\n";
      o << "        switch (e.field_index) {\n";
      for (size_t fi = 0; fi < msg.fields.size(); ++fi) {
        const FieldInfo & f = msg.fields[fi];
        const std::string lhs = rpc.snake_name + "_rx_buf_." + f.name;
        o << "          case " << fi << ": {\n";
        o << gen_decode_field_from_eoffset(f, lhs);
        o << "            break;\n";
        o << "          }\n";
      }
      o << "          default: break;\n";
      o << "        }\n";
      o << "        " << rpc.snake_name << "_updated = true;\n";
      o << "        break;\n";
      o << "      }\n";
    }
    o << "      default: break;\n";
    o << "    }\n";
  }
  o << "  }\n";
  if (si.rx_topic_count > 0) {
    for (const auto & rpc : si.rpcs) {
      if (rpc.kind != RpcKind::RX_TOPIC) continue;
      o << "  if (" << rpc.snake_name << "_updated && " << rpc.snake_name << "_cb_) "
        << rpc.snake_name << "_cb_(" << rpc.snake_name << "_rx_buf_, "
        << rpc.snake_name << "_ctx_);\n";
    }
  }
  o << "}\n\n";

  // fill_pdo_tx
  o << "uint8_t Node::fill_pdo_tx(uint16_t pdo_id, uint8_t * buf, uint8_t max_len) {\n";
  o << "  uint8_t result_len = 0;\n";
  o << "  for (uint8_t i = 0; i < pdo_tx_count(); ++i) {\n";
  o << "    const auto & e = pdo_tx_at(i);\n";
  o << "    if (e.pdo_id != pdo_id) continue;\n";
  o << "    if (e.offset + e.size > max_len) continue;\n";
  if (si.tx_topic_count > 0) {
    o << "    switch (e.topic_index) {\n";
    for (const auto & rpc : si.rpcs) {
      if (rpc.kind != RpcKind::TX_TOPIC) continue;
      if (rpc.response_msg_idx < 0) continue;
      const MessageInfo & msg = si.messages[rpc.response_msg_idx];
      o << "      case " << rpc.index << ": {  // " << rpc.snake_name << "\n";
      o << "        switch (e.field_index) {\n";
      for (size_t fi = 0; fi < msg.fields.size(); ++fi) {
        const FieldInfo & f = msg.fields[fi];
        const std::string src = rpc.snake_name + "_buf_." + f.name;
        o << "          case " << fi << ": {\n";
        o << gen_encode_field_to_eoffset(f, src);
        o << "            break;\n";
        o << "          }\n";
      }
      o << "          default: break;\n";
      o << "        }\n";
      o << "        { uint8_t _end = static_cast<uint8_t>(e.offset + e.size);\n";
      o << "          if (_end > result_len) result_len = _end; }\n";
      o << "        break;\n";
      o << "      }\n";
    }
    o << "      default: break;\n";
    o << "    }\n";
  }
  o << "  }\n";
  o << "  return result_len;\n";
  o << "}\n\n";

  // on_param_read
  o << "protocan::Status Node::on_param_read(uint8_t idx, uint8_t * out, uint8_t & out_size) {\n";
  if (si.param_count > 0) {
    o << "  switch (idx) {\n";
    for (const auto & rpc : si.rpcs) {
      if (rpc.kind != RpcKind::PARAM_RW && rpc.kind != RpcKind::PARAM_RO) continue;
      o << "    case " << rpc.index << ":  // " << rpc.snake_name << "\n";
      o << "      if (" << rpc.snake_name
        << "_get_cb_) return " << rpc.snake_name
        << "_get_cb_(out, out_size, " << rpc.snake_name << "_get_ctx_);\n";
      o << "      return protocan::Status::NOT_FOUND;\n";
    }
    o << "    default: return protocan::Status::NOT_FOUND;\n";
    o << "  }\n";
  } else {
    o << "  (void)idx; (void)out; (void)out_size;\n";
    o << "  return protocan::Status::NOT_FOUND;\n";
  }
  o << "}\n\n";

  // on_param_write
  o << "protocan::Status Node::on_param_write("
       "uint8_t idx, const uint8_t * data, uint8_t sz) {\n";
  bool has_rw_param = false;
  for (const auto & rpc : si.rpcs) {
    if (rpc.kind == RpcKind::PARAM_RW) { has_rw_param = true; break; }
  }
  if (has_rw_param) {
    o << "  switch (idx) {\n";
    for (const auto & rpc : si.rpcs) {
      if (rpc.kind != RpcKind::PARAM_RW) continue;
      o << "    case " << rpc.index << ":  // " << rpc.snake_name << "\n";
      o << "      if (" << rpc.snake_name
        << "_set_cb_) return " << rpc.snake_name
        << "_set_cb_(data, sz, " << rpc.snake_name << "_set_ctx_);\n";
      o << "      return protocan::Status::NOT_FOUND;\n";
    }
    o << "    default: return protocan::Status::NOT_FOUND;\n";
    o << "  }\n";
  } else {
    o << "  (void)idx; (void)data; (void)sz;\n";
    o << "  return protocan::Status::NOT_FOUND;\n";
  }
  o << "}\n\n";

  // on_service_req
  o << "protocan::Status Node::on_service_req(\n";
  o << "  uint8_t svc, const uint8_t * req, uint8_t rsz, uint8_t * res, uint8_t & osz) {\n";
  if (si.service_count > 0) {
    o << "  switch (svc) {\n";
    for (const auto & rpc : si.rpcs) {
      if (rpc.kind != RpcKind::SERVICE) continue;
      std::string req_name = (rpc.request_msg_idx >= 0)
        ? si.messages[rpc.request_msg_idx].proto_name : "";
      std::string res_name = (rpc.response_msg_idx >= 0)
        ? si.messages[rpc.response_msg_idx].proto_name : "";
      int req_size = (rpc.request_msg_idx >= 0)
        ? si.messages[rpc.request_msg_idx].packed_size : 0;
      int res_size = (rpc.response_msg_idx >= 0)
        ? si.messages[rpc.response_msg_idx].packed_size : 0;

      o << "    case " << rpc.index << ": {  // " << rpc.snake_name << "\n";
      o << "      if (!" << rpc.snake_name << "_cb_) return protocan::Status::NOT_FOUND;\n";
      if (!req_name.empty() && req_size > 0) {
        o << "      if (rsz < " << req_name << "::PACKED_SIZE)"
             " return protocan::Status::INVALID_ARGUMENT;\n";
        o << "      auto req_msg = " << req_name << "::decode(req);\n";
      }
      if (!res_name.empty()) {
        o << "      " << res_name << " res_msg{};\n";
        if (!req_name.empty() && req_size > 0) {
          o << "      protocan::Status s = " << rpc.snake_name
            << "_cb_(req_msg, res_msg, " << rpc.snake_name << "_ctx_);\n";
        } else {
          // empty request
          std::string dummy_req = req_name.empty() ? "" : (req_name + " req_msg{}; ");
          if (!req_name.empty()) {
            o << "      " << req_name << " req_msg{};\n";
          }
          o << "      protocan::Status s = " << rpc.snake_name << "_cb_("
            << (req_name.empty() ? "" : "req_msg, ") << "res_msg, "
            << rpc.snake_name << "_ctx_);\n";
        }
        o << "      if (s == protocan::Status::OK && "
          << res_name << "::PACKED_SIZE <= 62) {\n";
        o << "        res_msg.encode(res);\n";
        o << "        osz = static_cast<uint8_t>(" << res_name << "::PACKED_SIZE);\n";
        o << "      }\n";
        o << "      return s;\n";
      } else {
        o << "      return protocan::Status::NOT_FOUND;\n";
      }
      o << "    }\n";
    }
    o << "    default: return protocan::Status::NOT_FOUND;\n";
    o << "  }\n";
  } else {
    o << "  (void)svc; (void)req; (void)rsz; (void)res; (void)osz;\n";
    o << "  return protocan::Status::NOT_FOUND;\n";
  }
  o << "}\n\n";

  o << ns_close(si.package_name);
  return o.str();
}

// ─── Plugin entry ─────────────────────────────────────────────────────────────

bool ProtocAnGenerator::Generate(
  const google::protobuf::FileDescriptor *       file,
  const std::string & /*parameter*/,
  google::protobuf::compiler::GeneratorContext * context,
  std::string *                                  error) const
{
  for (int i = 0; i < file->service_count(); ++i) {
    const google::protobuf::ServiceDescriptor * svc = file->service(i);

    ServiceInfo si;
    if (!analyze_service(file, svc, si, error)) {
      return false;
    }

    const std::string base = si.snake_name;

    {
      std::unique_ptr<google::protobuf::io::ZeroCopyOutputStream> os(
        context->Open(base + ".hpp"));
      write_string(os.get(), gen_hpp(si));
    }
    {
      std::unique_ptr<google::protobuf::io::ZeroCopyOutputStream> os(
        context->Open(base + ".cpp"));
      write_string(os.get(), gen_cpp(si));
    }
  }
  return true;
}

}  // namespace protocan_gen
