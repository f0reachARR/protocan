#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros_babel_fish/detail/babel_fish_publisher.hpp>
#include <ros_babel_fish/detail/babel_fish_service.hpp>
#include <ros_babel_fish/detail/babel_fish_subscription.hpp>

#include "protocan/descriptor_parser.hpp"

struct TopicHandle
{
  uint16_t pdo_id = 0;  // For RX topics: PDO ID to send; set after PDO mapping generation
  ros_babel_fish::BabelFishPublisher::SharedPtr publisher;       // TX (device→ROS)
  ros_babel_fish::BabelFishSubscription::SharedPtr subscription; // RX (ROS→device)
};

struct NodeHandler
{
  protocan::ParsedDescriptor desc;
  std::string ros_ns;
  uint8_t device_id = 0;
  uint8_t local_node_id = 0;
  std::unordered_map<uint32_t, TopicHandle> tx_topics;  // topic_index → TopicHandle
  std::unordered_map<uint32_t, TopicHandle> rx_topics;  // topic_index → TopicHandle
  std::vector<ros_babel_fish::BabelFishService::SharedPtr> service_servers;
};
