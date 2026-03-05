#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <unordered_map>

#include "protocan/bulk_transfer.hpp"
#include "protocan/can_frame.hpp"
#include "protocan/can_interface.hpp"
#include "protocan/descriptor_parser.hpp"
#include "protocan/device_tracker.hpp"
#include "protocan/pdo_manager.hpp"
#include "protocan/types.hpp"

namespace protocan
{

// ════════════════════════════════════════════════════════════════
// マスター側コールバック (上位レイヤーへの通知)
// ════════════════════════════════════════════════════════════════

/// マスターイベントコールバック
struct MasterCallbacks
{
  /// 新しいデバイスが発見された
  std::function<void(uint8_t device_id, const DeviceInfo & info)> on_device_discovered;

  /// デバイスがタイムアウトした（Heartbeat 途絶）
  std::function<void(uint8_t device_id)> on_device_timeout;

  /// ディスクリプタが取得・パースされた
  std::function<void(uint8_t device_id, uint8_t local_node_id, const ParsedDescriptor & desc)>
    on_descriptor_received;

  /// PDO データを受信した (Standard ID)
  std::function<void(uint16_t pdo_id, const uint8_t * data, uint8_t len)> on_pdo_received;

  /// EMCY メッセージを受信した
  std::function<void(uint8_t device_id, uint8_t node_id, const EmcyMessage & emcy)>
    on_emcy_received;

  /// Service Response を受信した
  std::function<void(
    uint8_t device_id, uint8_t node_id, uint8_t service_index, ServiceStatus status,
    const uint8_t * data, size_t len)>
    on_service_response;

  /// Parameter Read/Write Response を受信した
  std::function<void(
    uint8_t device_id, uint8_t node_id, ParamCommand cmd, uint8_t param_index, ParamStatus status,
    const uint8_t * data, size_t len)>
    on_param_response;
};

// ════════════════════════════════════════════════════════════════
// マスターオーケストレータ
// ════════════════════════════════════════════════════════════════

/// ProtoCAN マスター統合クラス
///
/// CAN バス上のフレームを受信・ディスパッチし、
/// デバイス発見・ディスクリプタ取得・PDO 設定・サービス呼び出し等を
/// 一元管理する。
class Master
{
public:
  /// @param can_if  CAN バスインターフェース (所有権は呼び出し側)
  /// @param callbacks  イベントコールバック群
  explicit Master(ICanInterface & can_if, MasterCallbacks callbacks = {});

  ~Master();

  // ── メインループ ──

  /// 1 回の受信ポーリング＆フレーム処理
  /// ブリッジのメインループから繰り返し呼ぶ。
  void poll();

  /// タイマー駆動の定期処理（タイムアウト検出等）
  /// @param now  現在時刻
  void tick(std::chrono::steady_clock::time_point now);

  // ── NMT 制御 ──

  /// NMT コマンドを送信する (spec §5.1)
  Status send_nmt_ctrl(uint8_t target_device_id, NmtCommand command);

  /// 全デバイスに ENTER_PREOP を送信する（マスター再起動時リカバリ）
  Status broadcast_enter_preop();

  // ── DISC ──

  /// ディスクリプタ取得要求を送信する (spec §5.3)
  Status send_disc_get_descriptor(uint8_t target_device_id, uint8_t target_node_id);

  // ── PARAM ──

  /// パラメータ READ 要求を送信する (spec §5.4)
  Status send_param_read(uint8_t target_device_id, uint8_t target_node_id, uint8_t param_index);

  /// パラメータ WRITE 要求を送信する (spec §5.4)
  Status send_param_write(
    uint8_t target_device_id, uint8_t target_node_id, uint8_t param_index, const uint8_t * value,
    uint8_t value_len);

  // ── PDO_CFG ──

  /// PDO マッピングをデバイスに送信する (spec §5.5)
  /// BEGIN → ENTRY × N → COMMIT の一連をまとめて送信。
  Status send_pdo_cfg(uint8_t target_device_id, const PdoMapping & mapping);

  /// PDO マッピングを削除する
  Status send_pdo_cfg_delete(uint8_t target_device_id, uint16_t pdo_id);

  // ── SERVICE ──

  /// Service Request を送信する (spec §5.8)
  /// @return sequence_id (応答照合用)
  std::optional<uint8_t> send_service_request(
    uint8_t target_device_id, uint8_t target_node_id, uint8_t service_index,
    const uint8_t * request_data, size_t request_len);

  // ── アクセサ ──

  DeviceTracker & device_tracker() { return tracker_; }
  const DeviceTracker & device_tracker() const { return tracker_; }

  PdoManager & pdo_manager() { return pdo_mgr_; }
  const PdoManager & pdo_manager() const { return pdo_mgr_; }

private:
  // ── フレームディスパッチ ──
  void process_frame(const CanFrame & frame);
  void process_management_frame(const CanFrame & frame);
  void process_pdo_frame(const CanFrame & frame);

  // ── Function Code ハンドラ ──
  void handle_nmt(const ExtendedId & eid, const uint8_t * data, uint8_t len);
  void handle_emcy(const ExtendedId & eid, const uint8_t * data, uint8_t len);
  void handle_disc(const ExtendedId & eid, const uint8_t * data, uint8_t len);
  void handle_param(const ExtendedId & eid, const uint8_t * data, uint8_t len);
  void handle_pdo_cfg(const ExtendedId & eid, const uint8_t * data, uint8_t len);
  void handle_bulk(const ExtendedId & eid, const uint8_t * data, uint8_t len);
  void handle_service(const ExtendedId & eid, const uint8_t * data, uint8_t len);

  // ── BULK 転送完了ハンドラ ──
  void on_bulk_complete(const ExtendedId & eid, const BulkReceiver & receiver);

  // ── ヘルパー ──
  Status send_frame(const CanFrame & frame);

  // ── メンバ ──
  ICanInterface & can_if_;
  MasterCallbacks callbacks_;
  DeviceTracker tracker_;
  PdoManager pdo_mgr_;
  BulkChannelManager bulk_channels_;

  /// BULK 受信セッション: (src_dev, src_node, channel) → BulkReceiver
  struct BulkRxKey
  {
    uint8_t src_dev;
    uint8_t src_node;
    uint8_t channel;
    bool operator==(const BulkRxKey & o) const
    {
      return src_dev == o.src_dev && src_node == o.src_node && channel == o.channel;
    }
  };
  struct BulkRxKeyHash
  {
    size_t operator()(const BulkRxKey & k) const
    {
      return (static_cast<size_t>(k.src_dev) << 16) | (static_cast<size_t>(k.src_node) << 8) |
             k.channel;
    }
  };
  std::unordered_map<BulkRxKey, BulkReceiver, BulkRxKeyHash> bulk_receivers_;

  /// Service 未応答の sequence_id トラッキング
  struct PendingService
  {
    uint8_t device_id;
    uint8_t node_id;
    uint8_t service_index;
    std::chrono::steady_clock::time_point sent_at;
  };
  std::unordered_map<uint8_t, PendingService> pending_services_;  // seq_id → info
  uint8_t next_service_seq_ = 0;

  /// Heartbeat タイムアウト閾値
  std::chrono::milliseconds heartbeat_timeout_{3000};
};

}  // namespace protocan
