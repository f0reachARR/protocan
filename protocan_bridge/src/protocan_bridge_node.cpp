#include "protocan_bridge_node.hpp"

#include <cctype>
#include <stdexcept>

#include "protocan/can_frame.hpp"
#include "protocan/packed_binary.hpp"

// ════════════════════════════════════════════════════════════════
// Internal helpers
// ════════════════════════════════════════════════════════════════

// Convert "BLDCMotor" → "bldc_motor"
static std::string camel_to_snake(const std::string & input)
{
  std::string result;
  result.reserve(input.size() + 4);
  for (size_t i = 0; i < input.size(); ++i) {
    char c = input[i];
    if (std::isupper(static_cast<unsigned char>(c))) {
      bool prev_lower = (i > 0 && std::islower(static_cast<unsigned char>(input[i - 1])));
      bool next_lower = (i + 1 < input.size() &&
                         std::islower(static_cast<unsigned char>(input[i + 1])));
      bool prev_upper = (i > 0 && std::isupper(static_cast<unsigned char>(input[i - 1])));
      if (i > 0 && (prev_lower || (next_lower && prev_upper))) {
        result += '_';
      }
      result += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    } else {
      result += c;
    }
  }
  return result;
}

// Navigate a dot-separated field path in a CompoundMessage (mutable)
static ros_babel_fish::Message & navigate_to_field(
  ros_babel_fish::CompoundMessage & msg, const std::string & path)
{
  auto dot = path.find('.');
  if (dot == std::string::npos) {
    return msg[path];
  }
  return navigate_to_field(
    msg[path.substr(0, dot)].as<ros_babel_fish::CompoundMessage>(),
    path.substr(dot + 1));
}

// Navigate a dot-separated field path in a CompoundMessage (const)
static const ros_babel_fish::Message & navigate_to_field(
  const ros_babel_fish::CompoundMessage & msg, const std::string & path)
{
  auto dot = path.find('.');
  if (dot == std::string::npos) {
    return msg[path];
  }
  return navigate_to_field(
    msg[path.substr(0, dot)].as<ros_babel_fish::CompoundMessage>(),
    path.substr(dot + 1));
}

// Read a FieldValue from a Message leaf based on ProtoCAN field_type enum
static protocan::FieldValue field_value_from_message(
  const ros_babel_fish::Message & msg, uint8_t field_type)
{
  switch (field_type) {
    case 0: return msg.value<bool>();
    case 1: return msg.value<uint8_t>();
    case 2: return msg.value<int8_t>();
    case 3: return msg.value<uint16_t>();
    case 4: return msg.value<int16_t>();
    case 5: return msg.value<uint32_t>();
    case 6: return msg.value<int32_t>();
    case 7: return msg.value<float>();
    case 8: return msg.value<double>();
    case 9: return msg.value<uint64_t>();
    case 10: return msg.value<int64_t>();
    default: return uint32_t{0};
  }
}

// ════════════════════════════════════════════════════════════════
// Static callback factory
// ════════════════════════════════════════════════════════════════

protocan::MasterCallbacks ProtoCanbridgeNode::make_callbacks(ProtoCanbridgeNode * node)
{
  protocan::MasterCallbacks cbs;

  cbs.on_device_discovered = [node](uint8_t dev, const protocan::DeviceInfo & info) {
    node->on_device_discovered(dev, info);
  };

  cbs.on_device_timeout = [node](uint8_t dev) {
    node->on_device_timeout(dev);
  };

  cbs.on_descriptor_received = [node](
    uint8_t dev, uint8_t node_id, const protocan::ParsedDescriptor & desc) {
    node->on_descriptor_received(dev, node_id, desc);
  };

  cbs.on_pdo_data = [node](const protocan::PdoDecodedData & decoded) {
    node->on_pdo_data(decoded);
  };

  // Unused callbacks – leave as nullptr (MasterCallbacks checks before calling)
  return cbs;
}

// ════════════════════════════════════════════════════════════════
// Constructor / Destructor
// ════════════════════════════════════════════════════════════════

ProtoCanbridgeNode::ProtoCanbridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("protocan_bridge", options),
  can_if_(declare_parameter<std::string>("can_interface", "can0")),
  master_(can_if_, make_callbacks(this))
{
  if (can_if_.open() != protocan::Status::OK) {
    RCLCPP_ERROR(get_logger(), "Failed to open CAN interface '%s'",
                 get_parameter("can_interface").as_string().c_str());
  } else {
    RCLCPP_INFO(get_logger(), "CAN interface '%s' opened",
                get_parameter("can_interface").as_string().c_str());
  }

  poll_timer_ = create_wall_timer(
    std::chrono::milliseconds(1), [this]() { on_poll(); });

  tick_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), [this]() { on_tick(); });
}

ProtoCanbridgeNode::~ProtoCanbridgeNode()
{
  can_if_.close();
}

// ════════════════════════════════════════════════════════════════
// Timer callbacks
// ════════════════════════════════════════════════════════════════

void ProtoCanbridgeNode::on_poll()
{
  master_.poll();

  // Request descriptors for any newly-seen, unknown schema_hashes
  for (auto & [hash, dev, node] : master_.device_tracker().collect_unknown_schemas()) {
    (void)hash;
    master_.send_disc_get_descriptor(dev, node);
  }
}

void ProtoCanbridgeNode::on_tick()
{
  master_.tick(std::chrono::steady_clock::now());
}

// ════════════════════════════════════════════════════════════════
// Device lifecycle callbacks
// ════════════════════════════════════════════════════════════════

void ProtoCanbridgeNode::on_device_discovered(
  uint8_t device_id, const protocan::DeviceInfo & /*info*/)
{
  RCLCPP_INFO(get_logger(), "Device %u discovered", device_id);
}

void ProtoCanbridgeNode::on_device_timeout(uint8_t device_id)
{
  RCLCPP_WARN(get_logger(), "Device %u timed out – removing handlers", device_id);

  // Collect keys belonging to this device
  std::vector<uint32_t> to_remove;
  for (auto & [key, handler] : handlers_) {
    if (handler.device_id == device_id) {
      to_remove.push_back(key);
      // Release PDO allocations for RX topics
      for (auto & [tidx, th] : handler.topics) {
        if (!th.is_tx && th.pdo_id != 0) {
          master_.pdo_manager().release(th.pdo_id);
          pdo_rx_buffers_.erase(th.pdo_id);
        }
      }
    }
  }
  for (auto key : to_remove) {
    handlers_.erase(key);
  }
}

// ════════════════════════════════════════════════════════════════
// Descriptor received – main setup
// ════════════════════════════════════════════════════════════════

void ProtoCanbridgeNode::on_descriptor_received(
  uint8_t device_id, uint8_t local_node_id, const protocan::ParsedDescriptor & desc)
{
  const uint32_t hkey = (static_cast<uint32_t>(device_id) << 8) | local_node_id;
  RCLCPP_INFO(get_logger(), "Descriptor received for device %u node %u (%s)",
              device_id, local_node_id, desc.node_type_name.c_str());

  // Remove old handler if re-registered
  handlers_.erase(hkey);

  // ── Determine ROS namespace ──
  std::string ros_ns;
  if (!desc.ros2_namespace.empty()) {
    ros_ns = desc.ros2_namespace;
  } else {
    ros_ns = "/protocan/device_" + std::to_string(device_id) + "/" +
             camel_to_snake(desc.node_type_name);
  }
  // Ensure trailing slash
  if (!ros_ns.empty() && ros_ns.back() != '/') {
    ros_ns += '/';
  }

  // ── Create handler entry ──
  NodeHandler & handler = handlers_[hkey];
  handler.desc = desc;
  handler.ros_ns = ros_ns;
  handler.device_id = device_id;
  handler.local_node_id = local_node_id;

  // ── Create TX publishers (device → ROS) ──
  for (auto & topic : desc.topics) {
    if (!topic.is_tx) continue;
    const std::string full_topic = ros_ns + topic.name;
    auto pub = babel_fish_.create_publisher(
      *this, full_topic, topic.message.ros2_msg_type, rclcpp::QoS(10));
    TopicHandle th;
    th.is_tx = true;
    th.publisher = pub;
    handler.topics[topic.index] = std::move(th);
    RCLCPP_INFO(get_logger(), "  TX publisher: %s [%s]",
                full_topic.c_str(), topic.message.ros2_msg_type.c_str());
  }

  // ── Generate and send PDO mappings ──
  protocan::NodeConfig cfg{local_node_id, &handler.desc};
  auto mappings = master_.pdo_manager().generate_optimal_mappings(device_id, {cfg});

  for (auto & m : mappings) {
    master_.send_pdo_cfg(device_id, m);
    master_.pdo_manager().set_mapping(m);

    // For RX PDOs: record pdo_id per topic_index
    if (m.direction == protocan::PdoCfgDirection::RX) {
      for (auto & entry : m.entries) {
        auto it = handler.topics.find(entry.topic_index);
        if (it != handler.topics.end() && !it->second.is_tx) {
          it->second.pdo_id = m.pdo_id;
        }
      }
    }
  }

  // ── Create RX subscriptions (ROS → device) ──
  for (auto & topic : desc.topics) {
    if (topic.is_tx) continue;
    const std::string full_topic = ros_ns + topic.name;
    const uint32_t tidx = topic.index;

    auto sub = babel_fish_.create_subscription(
      *this, full_topic, topic.message.ros2_msg_type, rclcpp::QoS(10),
      [this, hkey, tidx](ros_babel_fish::CompoundMessage::ConstSharedPtr msg) {
        on_rx_topic(hkey, tidx, *msg);
      });

    // The handler.topics entry may already exist (pdo_id was set above)
    auto & th = handler.topics[tidx];
    th.is_tx = false;
    th.subscription = sub;
    RCLCPP_INFO(get_logger(), "  RX subscription: %s [%s]",
                full_topic.c_str(), topic.message.ros2_msg_type.c_str());
  }

  // ── Create service servers (ROS → device) ──
  for (auto & svc : desc.services) {
    const std::string svc_name = ros_ns + svc.name;
    const uint8_t svc_idx = static_cast<uint8_t>(svc.index);

    auto server = babel_fish_.create_service(
      *this, svc_name, svc.ros2_srv_type,
      [this, device_id, local_node_id, svc_idx, &svc](
        std::shared_ptr<ros_babel_fish::CompoundMessage> req,
        std::shared_ptr<ros_babel_fish::CompoundMessage> /*res*/)
      {
        // Encode request fields into a byte buffer
        std::array<uint8_t, protocan::kCanFdMaxPayload> buf{};
        for (size_t fi = 0; fi < svc.request.fields.size(); ++fi) {
          auto & fdesc = svc.request.fields[fi];
          if (fdesc.ros2_field.empty()) continue;
          try {
            const auto & leaf = navigate_to_field(
              req->as<ros_babel_fish::CompoundMessage>(), fdesc.ros2_field);
            auto fv = field_value_from_message(leaf, fdesc.type);
            protocan::encode_field(buf.data(), fdesc.offset, fdesc.type, fv, fdesc.size);
          } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "Service request encoding error: %s", e.what());
          }
        }
        master_.send_service_request(
          device_id, local_node_id, svc_idx, buf.data(), svc.request.payload_size);
      });

    handler.service_servers.push_back(server);
    RCLCPP_INFO(get_logger(), "  Service server: %s [%s]",
                svc_name.c_str(), svc.ros2_srv_type.c_str());
  }

  // Transition device to OPERATIONAL so TX PDOs start flowing
  master_.send_nmt_ctrl(device_id, protocan::NmtCommand::START);
}

// ════════════════════════════════════════════════════════════════
// PDO decoded data received (device → ROS)
// ════════════════════════════════════════════════════════════════

void ProtoCanbridgeNode::on_pdo_data(const protocan::PdoDecodedData & decoded)
{
  // Group decoded fields by (device_id, local_node_id, topic_index)
  std::unordered_map<uint32_t, std::vector<const protocan::PdoDecodedField *>> groups;
  for (auto & f : decoded.fields) {
    uint32_t gkey = (static_cast<uint32_t>(f.device_id) << 16) |
                    (static_cast<uint32_t>(f.local_node_id) << 8) |
                    f.topic_index;
    groups[gkey].push_back(&f);
  }

  for (auto & [gkey, fields] : groups) {
    const uint8_t dev = static_cast<uint8_t>(gkey >> 16);
    const uint8_t nid = static_cast<uint8_t>((gkey >> 8) & 0xFF);
    const uint8_t tidx = static_cast<uint8_t>(gkey & 0xFF);

    const uint32_t hkey = (static_cast<uint32_t>(dev) << 8) | nid;
    auto hit = handlers_.find(hkey);
    if (hit == handlers_.end()) continue;
    NodeHandler & handler = hit->second;

    auto tit = handler.topics.find(tidx);
    if (tit == handler.topics.end() || !tit->second.is_tx) continue;
    auto & pub = tit->second.publisher;
    if (!pub) continue;

    // Find topic descriptor
    const protocan::ParsedTopic * ptopic = nullptr;
    for (auto & t : handler.desc.topics) {
      if (t.index == tidx) { ptopic = &t; break; }
    }
    if (!ptopic) continue;

    // Build message and fill fields
    ros_babel_fish::CompoundMessage msg =
      babel_fish_.create_message(ptopic->message.ros2_msg_type);

    for (auto * f : fields) {
      if (f->field_index >= ptopic->message.fields.size()) continue;
      const auto & fdesc = ptopic->message.fields[f->field_index];
      if (fdesc.ros2_field.empty()) continue;
      try {
        auto & leaf = navigate_to_field(msg, fdesc.ros2_field);
        std::visit([&leaf](auto v) { leaf = v; }, f->value);
      } catch (const std::exception & e) {
        RCLCPP_WARN_ONCE(get_logger(), "PDO field set error: %s", e.what());
      }
    }

    pub->publish(msg);
  }
}

// ════════════════════════════════════════════════════════════════
// RX topic received (ROS → device)
// ════════════════════════════════════════════════════════════════

void ProtoCanbridgeNode::on_rx_topic(
  uint32_t handler_key, uint32_t topic_index, const ros_babel_fish::CompoundMessage & msg)
{
  auto hit = handlers_.find(handler_key);
  if (hit == handlers_.end()) return;
  NodeHandler & handler = hit->second;

  auto tit = handler.topics.find(topic_index);
  if (tit == handler.topics.end() || tit->second.is_tx) return;
  const uint16_t pdo_id = tit->second.pdo_id;
  if (pdo_id == 0) return;

  // Find topic descriptor
  const protocan::ParsedTopic * ptopic = nullptr;
  for (auto & t : handler.desc.topics) {
    if (t.index == topic_index) { ptopic = &t; break; }
  }
  if (!ptopic) return;

  // Get PDO mapping
  auto maybe_map = master_.pdo_manager().get_mapping(pdo_id);
  if (!maybe_map) return;
  const auto & mapping = *maybe_map;

  // Get or create the PDO buffer
  auto & buf = pdo_rx_buffers_[pdo_id];
  buf.fill(0);  // zero out before filling (keeps other-topic fields at zero)

  // Encode fields from the ROS message into the PDO buffer
  for (auto & entry : mapping.entries) {
    if (entry.local_node_id != handler.local_node_id) continue;
    if (entry.topic_index != static_cast<uint8_t>(topic_index)) continue;
    if (entry.field_index >= ptopic->message.fields.size()) continue;

    const auto & fdesc = ptopic->message.fields[entry.field_index];
    if (fdesc.ros2_field.empty()) continue;

    try {
      const auto & leaf = navigate_to_field(msg, fdesc.ros2_field);
      auto fv = field_value_from_message(leaf, fdesc.type);
      protocan::encode_field(buf.data(), entry.offset, fdesc.type, fv, entry.size);
    } catch (const std::exception & e) {
      RCLCPP_WARN_ONCE(get_logger(), "RX topic encode error: %s", e.what());
    }
  }

  // Send the PDO frame
  auto frame = protocan::make_standard_frame(pdo_id, buf.data(), mapping.total_size);
  can_if_.send(frame);
}
