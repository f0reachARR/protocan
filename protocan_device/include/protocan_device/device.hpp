#pragma once

#include <cstddef>
#include <cstdint>

#include "protocan/can_frame.hpp"
#include "protocan/types.hpp"
#include "protocan_device/can_interface.hpp"
#include "protocan_device/node_base.hpp"

namespace protocan::device
{

static constexpr uint8_t kMaxNodes = 16;
static constexpr uint32_t kHeartbeatIntervalPreopMs = 100;
static constexpr uint32_t kHeartbeatIntervalOpMs = 1000;
static constexpr uint32_t kListenOnlyMinMs = 100;
static constexpr uint32_t kListenOnlyMaxMs = 500;
static constexpr uint8_t kInitialHbWatchCount = 3;

using GetTimeMsFn = uint32_t (*)();
using RandMsFn = uint32_t (*)(uint32_t min_ms, uint32_t max_ms);

class Device
{
public:
  /// @param device_id    DIPスイッチ等で設定した1-14の物理ID
  /// @param can          CAN HAL実装へのポインタ
  /// @param get_time_ms  現在時刻(ms)取得関数
  /// @param rand_ms      [min,max]範囲の乱数(ms)生成関数
  Device(
    uint8_t device_id, ICanInterface * can, GetTimeMsFn get_time_ms, RandMsFn rand_ms);

  /// ノード追加 (start()前に呼ぶ)
  Status add_node(NodeBase & node);

  /// Listen-Only期間開始 → PREOP遷移
  void start();

  /// メインループから呼ぶ (受信/ハートビート/PDO TX/BULK送信継続)
  void poll();

  /// 緊急通知送信
  Status send_emcy(
    uint8_t local_node_id, uint16_t error_register, const uint8_t * error_data,
    uint8_t error_data_size);

  DeviceState state() const { return state_; }

private:
  uint8_t device_id_;
  ICanInterface * can_;
  GetTimeMsFn get_time_ms_;
  RandMsFn rand_ms_;
  DeviceState state_ = DeviceState::PREOP;
  uint32_t start_ms_ = 0;
  uint32_t last_heartbeat_ms_ = 0;
  bool listen_only_ = true;
  uint32_t listen_only_end_ms_ = 0;
  uint8_t initial_hb_count_ = 0;

  NodeBase * nodes_[kMaxNodes];
  uint8_t node_count_ = 0;

  // BULK TX セッション (descriptor or service_res 送信用)
  struct BulkTxSession
  {
    bool active = false;
    uint8_t dst_dev = 0;
    uint8_t dst_node = 0;
    uint8_t channel = 0;
    BulkPayloadType payload_type = BulkPayloadType::DESCRIPTOR;
    const uint8_t * data = nullptr;
    uint32_t total = 0;
    uint32_t sent = 0;
    uint8_t sequence = 1;
    bool waiting_fc = false;
  } bulk_tx_;

  // PDO_CFG トランザクション受信バッファ
  struct PdoCfgPending
  {
    bool active = false;
    uint16_t pdo_id = 0;
    PdoCfgDirection dir = PdoCfgDirection::TX;
    uint8_t num_entries = 0;
    uint16_t period_ms = 0;
    uint8_t total_size = 0;
    uint8_t received = 0;
    PdoCfgEntry entries[30];  // max 0x1E entries
  } pdo_cfg_pending_;

  // --- 内部ハンドラ ---
  void handle_frame(const CanFrame & f);
  void handle_nmt(const ExtendedId & eid, const CanFrame & f);
  void handle_disc(const ExtendedId & eid, const CanFrame & f);
  void handle_param(const ExtendedId & eid, const CanFrame & f);
  void handle_pdo_cfg(const ExtendedId & eid, const CanFrame & f);
  void handle_bulk(const ExtendedId & eid, const CanFrame & f);
  void handle_service(const ExtendedId & eid, const CanFrame & f);
  void handle_pdo(const CanFrame & f);

  void send_heartbeat();
  void start_bulk_tx(
    uint8_t dst_dev, uint8_t dst_node, uint8_t channel, BulkPayloadType payload_type,
    const uint8_t * blob, uint32_t len);
  void continue_bulk_tx();

  PdoCfgStatus commit_pdo_cfg();

  NodeBase * find_node(uint8_t local_node_id);
  Status send_frame(const CanFrame & f);
};

}  // namespace protocan::device
