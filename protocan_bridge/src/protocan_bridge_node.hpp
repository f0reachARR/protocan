#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>
#include <unordered_map>

#include "node_handler.hpp"
#include "protocan/master.hpp"
#include "socketcan_interface.hpp"

class ProtoCanbridgeNode : public rclcpp::Node
{
public:
  explicit ProtoCanbridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~ProtoCanbridgeNode() override;

private:
  // Timer callbacks
  void on_poll();
  void on_tick();

  // Master event callbacks
  void on_device_discovered(uint8_t device_id, const protocan::DeviceInfo & info);
  void on_device_timeout(uint8_t device_id);
  void on_descriptor_received(
    uint8_t device_id, uint8_t local_node_id, const protocan::ParsedDescriptor & desc);
  void on_pdo_data(const protocan::PdoDecodedData & decoded);

  // RX subscription callback: ROS → device
  void on_rx_topic(
    uint32_t handler_key, uint32_t topic_index, const ros_babel_fish::CompoundMessage & msg);

  // Helper: build MasterCallbacks pointing at this node
  static protocan::MasterCallbacks make_callbacks(ProtoCanbridgeNode * node);

  // Members (order matters for initializer list)
  SocketCanInterface can_if_;
  protocan::Master master_;
  ros_babel_fish::BabelFish babel_fish_;

  // (device_id << 8 | local_node_id) → NodeHandler
  std::unordered_map<uint32_t, NodeHandler> handlers_;

  // pdo_id → 64-byte PDO payload buffer for outgoing RX frames (ROS→device)
  std::map<uint16_t, std::array<uint8_t, 64>> pdo_rx_buffers_;

  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};
