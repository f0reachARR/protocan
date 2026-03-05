#include "protocan/device_tracker.hpp"

namespace protocan
{

void DeviceTracker::update_heartbeat(uint8_t device_id, const Heartbeat & heartbeat)
{
  auto & info = devices_[device_id];
  info.device_id = device_id;
  info.state = heartbeat.state;
  info.uptime_ms = heartbeat.uptime_ms;
  info.last_heartbeat = std::chrono::steady_clock::now();

  // PREOP 状態ではノード情報が含まれる。
  // OP 状態では num_nodes=0 で省略されるため、既存情報を保持する。
  if (heartbeat.num_nodes > 0) {
    info.nodes.clear();
    info.nodes.reserve(heartbeat.nodes.size());
    for (const auto & entry : heartbeat.nodes) {
      NodeInfo ni;
      ni.local_node_id = entry.local_node_id;
      ni.schema_hash = entry.schema_hash;
      info.nodes.push_back(ni);
    }
  }
}

std::optional<DeviceInfo> DeviceTracker::get_device(uint8_t device_id) const
{
  auto it = devices_.find(device_id);
  if (it == devices_.end()) {
    return std::nullopt;
  }
  return it->second;
}

bool DeviceTracker::is_schema_known(uint32_t schema_hash) const
{
  return schema_cache_.find(schema_hash) != schema_cache_.end();
}

void DeviceTracker::cache_descriptor(uint32_t schema_hash, ParsedDescriptor descriptor)
{
  schema_cache_[schema_hash] = std::move(descriptor);
}

std::optional<ParsedDescriptor> DeviceTracker::get_cached_descriptor(uint32_t schema_hash) const
{
  auto it = schema_cache_.find(schema_hash);
  if (it == schema_cache_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void DeviceTracker::invalidate_cache(uint32_t schema_hash) { schema_cache_.erase(schema_hash); }

std::vector<uint8_t> DeviceTracker::detect_timeouts(std::chrono::milliseconds timeout) const
{
  auto now = std::chrono::steady_clock::now();
  std::vector<uint8_t> timed_out;

  for (const auto & [dev_id, info] : devices_) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - info.last_heartbeat);
    if (elapsed > timeout) {
      timed_out.push_back(dev_id);
    }
  }

  return timed_out;
}

std::vector<std::tuple<uint32_t, uint8_t, uint8_t>> DeviceTracker::collect_unknown_schemas() const
{
  std::vector<std::tuple<uint32_t, uint8_t, uint8_t>> unknowns;

  // 既にリストに含まれた schema_hash は重複して返さない
  // （同じ hash をもつ複数ノードのうち 1 つだけ返す）
  std::unordered_map<uint32_t, bool> seen;

  for (const auto & [dev_id, info] : devices_) {
    for (const auto & node : info.nodes) {
      if (!is_schema_known(node.schema_hash) && seen.find(node.schema_hash) == seen.end()) {
        unknowns.emplace_back(node.schema_hash, dev_id, node.local_node_id);
        seen[node.schema_hash] = true;
      }
    }
  }

  return unknowns;
}

void DeviceTracker::clear()
{
  devices_.clear();
  schema_cache_.clear();
}

}  // namespace protocan
