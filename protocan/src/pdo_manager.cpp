#include "protocan/pdo_manager.hpp"

#include <map>
#include <tuple>
namespace protocan
{

std::optional<uint16_t> PdoManager::allocate(uint32_t priority)
{
  PdoPriority band = priority_to_band(priority);

  switch (band) {
    case PdoPriority::HIGH:
      return find_free_id(kPdoHighPriorityMin, kPdoHighPriorityMax);
    case PdoPriority::MID:
      return find_free_id(kPdoMidPriorityMin, kPdoMidPriorityMax);
    case PdoPriority::LOW:
      return find_free_id(kPdoLowPriorityMin, kPdoLowPriorityMax);
    default:
      return find_free_id(kPdoLowPriorityMin, kPdoLowPriorityMax);
  }
}

void PdoManager::release(uint16_t pdo_id)
{
  allocated_.erase(pdo_id);
  mappings_.erase(pdo_id);
}

void PdoManager::set_mapping(const PdoMapping & mapping)
{
  mappings_[mapping.pdo_id] = mapping;
  allocated_[mapping.pdo_id] = true;
}

std::optional<PdoMapping> PdoManager::get_mapping(uint16_t pdo_id) const
{
  auto it = mappings_.find(pdo_id);
  if (it == mappings_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void PdoManager::remove_mapping(uint16_t pdo_id)
{
  mappings_.erase(pdo_id);
  allocated_.erase(pdo_id);
}

void PdoManager::reset()
{
  mappings_.clear();
  allocated_.clear();
}

std::optional<uint16_t> PdoManager::find_free_id(uint16_t min_id, uint16_t max_id) const
{
  for (uint16_t id = min_id; id <= max_id; ++id) {
    if (allocated_.find(id) == allocated_.end()) {
      return id;
    }
  }
  return std::nullopt;
}

std::vector<PdoMapping> PdoManager::generate_optimal_mappings(
  uint8_t device_id, const std::vector<NodeConfig> & node_configs)
{
  std::vector<PdoMapping> results;

  // Grouping key: {direction (TX=0/RX=1), period_ms, priority}
  using GroupKey = std::tuple<PdoCfgDirection, uint32_t, uint32_t>;

  struct FieldRef
  {
    uint8_t local_node_id;
    uint8_t topic_index;
    uint8_t field_index;
    uint8_t size;
  };

  std::map<GroupKey, std::vector<FieldRef>> grouped_fields;

  // NodeConfigから全てのTopicの全てのFieldを収集・グループ化する
  for (const auto & nc : node_configs) {
    if (!nc.descriptor) continue;

    for (const auto & topic : nc.descriptor->topics) {
      PdoCfgDirection dir = topic.is_tx ? PdoCfgDirection::TX : PdoCfgDirection::RX;
      // 周期(period_ms)はオプションの "periodic" のみからは一意に決まらないが、
      // 簡単のため periodic=true の場合は 100ms としてグループ化する
      uint32_t period = topic.periodic ? 100 : 0;
      uint32_t priority = topic.priority;

      for (size_t f_idx = 0; f_idx < topic.message.fields.size(); ++f_idx) {
        const auto & f = topic.message.fields[f_idx];
        grouped_fields[{dir, period, priority}].push_back(
          {nc.local_node_id, static_cast<uint8_t>(topic.index), static_cast<uint8_t>(f_idx),
           static_cast<uint8_t>(f.size)});
      }
    }
  }

  // グループごとに 64 bytes に収まる範囲でパッキングする
  for (const auto & [key, fields] : grouped_fields) {
    PdoCfgDirection dir = std::get<0>(key);
    uint32_t period = std::get<1>(key);
    uint32_t priority = std::get<2>(key);

    PdoMapping current_mapping;
    current_mapping.direction = dir;
    current_mapping.period_ms = period;
    current_mapping.total_size = 0;

    auto pdo_id_opt = allocate(priority);
    if (!pdo_id_opt) continue;  // 空きがない場合はスキップ（異常系）
    current_mapping.pdo_id = *pdo_id_opt;

    for (const auto & f : fields) {
      // 1つの PDO のサイズ制限(64B)やエントリー制限(30個)を超える場合は、
      // 現在の PDO を確定させて次の PDO を割り当てる
      if (
        current_mapping.total_size + f.size > kCanFdMaxPayload ||
        current_mapping.entries.size() >= 30) {
        results.push_back(current_mapping);

        current_mapping.entries.clear();
        current_mapping.total_size = 0;
        auto next_id_opt = allocate(priority);
        if (!next_id_opt) break;
        current_mapping.pdo_id = *next_id_opt;
      }

      PdoMapping::Entry entry;
      entry.device_id = device_id;
      entry.local_node_id = f.local_node_id;
      entry.topic_index = f.topic_index;
      entry.field_index = f.field_index;
      entry.offset = current_mapping.total_size;  // PDO 内部の先頭から隙間なく詰める
      entry.size = f.size;

      current_mapping.entries.push_back(entry);
      current_mapping.total_size += f.size;
    }

    // 最後に未確定のエントリがあれば追加する
    if (!current_mapping.entries.empty()) {
      results.push_back(current_mapping);
    }
  }

  return results;
}

}  // namespace protocan
