#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "protocan/descriptor_parser.hpp"
#include "protocan/types.hpp"

namespace protocan
{

/// PDO 自動生成用のノード入力情報
struct NodeConfig
{
  uint8_t local_node_id;
  const ParsedDescriptor * descriptor;
};

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

  /// 複数のノード構成から最適な PDO マッピングのリストを自動生成する
  /// (同一方向、周期、優先度のトピックを集約して 64 バイト以内にパッキング)
  /// 生成された PDO は自動的に allocate されるが、set_mapping はされないため、
  /// 呼び出し側で send_pdo_cfg の後に set_mapping を行う必要がある。
  /// @param device_id      対象デバイス ID
  /// @param node_configs   ノードIDとディスクリプタのペアのリスト
  /// @return               生成された PdoMapping のリスト
  std::vector<PdoMapping> generate_optimal_mappings(
    uint8_t device_id, const std::vector<NodeConfig> & node_configs);

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
