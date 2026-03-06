#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "protocan/descriptor_parser.hpp"
#include "protocan/types.hpp"

namespace protocan
{

/// ノードインスタンスの情報
struct NodeInfo
{
  uint8_t local_node_id;
  uint32_t schema_hash;
};

/// デバイス情報
struct DeviceInfo
{
  uint8_t device_id;
  DeviceState state = DeviceState::PREOP;
  uint32_t uptime_ms = 0;
  std::vector<NodeInfo> nodes;
  std::chrono::steady_clock::time_point last_heartbeat;
};

/// デバイスおよびスキーマ状態を追跡するクラス
class DeviceTracker
{
public:
  /// Heartbeat を受信して情報を更新する
  /// @param device_id  デバイス ID
  /// @param heartbeat  パース済み Heartbeat データ
  void update_heartbeat(uint8_t device_id, const Heartbeat & heartbeat);

  /// 指定デバイスの情報を取得する
  std::optional<DeviceInfo> get_device(uint8_t device_id) const;

  /// 全デバイスの一覧を取得する
  const std::unordered_map<uint8_t, DeviceInfo> & devices() const { return devices_; }

  /// 指定の schema_hash が既知（キャッシュ済み）かどうか判定する
  bool is_schema_known(uint32_t schema_hash) const;

  /// ディスクリプタをキャッシュする
  void cache_descriptor(uint32_t schema_hash, ParsedDescriptor descriptor);

  /// キャッシュ済みディスクリプタを取得する (コピー)
  std::optional<ParsedDescriptor> get_cached_descriptor(uint32_t schema_hash) const;

  /// キャッシュ済みディスクリプタへのポインタを取得する (ゼロコピー)
  /// ホットパスでの利用向け。キャッシュが無効化されるとダングリングポインタになるため、
  /// 返却後にキャッシュ操作を行わないこと。
  const ParsedDescriptor * get_descriptor_ptr(uint32_t schema_hash) const;

  /// (device_id, local_node_id) からディスクリプタへのポインタを直接取得する
  /// 内部の逆引きインデックスを使用して O(1) で解決する。
  const ParsedDescriptor * get_node_descriptor(uint8_t device_id, uint8_t local_node_id) const;

  /// (device_id, local_node_id, topic_index) からトピックへのポインタを取得する
  const ParsedTopic * get_topic(
    uint8_t device_id, uint8_t local_node_id, uint8_t topic_index) const;

  /// (device_id, local_node_id, topic_index, field_index) からフィールドへのポインタを取得する
  const ParsedField * get_field(
    uint8_t device_id, uint8_t local_node_id, uint8_t topic_index, uint8_t field_index) const;

  /// Schema Hash キャッシュを無効化する (衝突時)
  void invalidate_cache(uint32_t schema_hash);

  /// タイムアウトしたデバイスを検出する (所定時間以上 Heartbeat なし)
  /// @param timeout  タイムアウト閾値
  /// @return タイムアウトとして検出された device_id のリスト
  std::vector<uint8_t> detect_timeouts(std::chrono::milliseconds timeout) const;

  /// 指定デバイスを追跡対象から削除する
  /// @return 削除した場合 true
  bool remove_device(uint8_t device_id);

  /// 全デバイスから未知の schema_hash を収集する
  /// @return 未知の (未キャッシュの) schema_hash と対応する (device_id, local_node_id) のペア
  std::vector<std::tuple<uint32_t, uint8_t, uint8_t>> collect_unknown_schemas() const;

  /// デバイス情報をクリアする
  void clear();

private:
  std::unordered_map<uint8_t, DeviceInfo> devices_;

  /// schema_hash → ParsedDescriptor のキャッシュ
  std::unordered_map<uint32_t, ParsedDescriptor> schema_cache_;

  /// (device_id, local_node_id) → schema_hash の逆引きインデックス
  struct NodeKey
  {
    uint8_t device_id;
    uint8_t local_node_id;
    bool operator==(const NodeKey & o) const
    {
      return device_id == o.device_id && local_node_id == o.local_node_id;
    }
  };
  struct NodeKeyHash
  {
    size_t operator()(const NodeKey & k) const
    {
      return (static_cast<size_t>(k.device_id) << 8) | k.local_node_id;
    }
  };
  std::unordered_map<NodeKey, uint32_t, NodeKeyHash> node_to_schema_;
};

}  // namespace protocan
