#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "protocan/types.hpp"

namespace protocan
{

/// PDO マッピングエントリ (1 つの Standard CAN ID に対する割当情報)
struct PdoMapping
{
  uint16_t pdo_id;
  PdoCfgDirection direction;
  uint16_t period_ms;
  uint8_t total_size;  // 合計バイトサイズ

  struct Entry
  {
    uint8_t device_id;
    uint8_t local_node_id;
    uint8_t topic_index;
    uint8_t field_index;
    uint8_t offset;
    uint8_t size;
  };
  std::vector<Entry> entries;
};

/// PDO Standard ID 割り当てと PDO マッピングの管理
class PdoManager
{
public:
  /// 指定優先度帯域から PDO ID を割り当てる (spec §4.3)
  /// @param priority  トピック優先度 (0-7)
  /// @return 割り当てられた PDO ID、空きがない場合は nullopt
  std::optional<uint16_t> allocate(uint32_t priority);

  /// PDO ID を解放する
  void release(uint16_t pdo_id);

  /// PDO マッピングを登録する
  void set_mapping(const PdoMapping & mapping);

  /// PDO マッピングを取得する
  std::optional<PdoMapping> get_mapping(uint16_t pdo_id) const;

  /// PDO マッピングを削除する
  void remove_mapping(uint16_t pdo_id);

  /// 全マッピングを取得する
  const std::unordered_map<uint16_t, PdoMapping> & mappings() const { return mappings_; }

  /// 全てリセットする
  void reset();

private:
  /// 優先度帯域から次の空き ID を検索
  std::optional<uint16_t> find_free_id(uint16_t min_id, uint16_t max_id) const;

  /// PDO ID → PdoMapping
  std::unordered_map<uint16_t, PdoMapping> mappings_;

  /// 割当済 PDO ID の集合 (高速ルックアップ用)
  std::unordered_map<uint16_t, bool> allocated_;
};

}  // namespace protocan
