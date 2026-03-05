#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>

#include "protocan/can_frame.hpp"
#include "protocan/types.hpp"

namespace protocan
{

// ════════════════════════════════════════════════════════════════
// BULK 受信ステートマシン (spec §5.6)
// ════════════════════════════════════════════════════════════════

/// BULK 転送の受信状態
enum class BulkRxState {
  IDLE,
  RECEIVING,
  COMPLETE,
  ERROR,
};

/// BULK 受信セッション
class BulkReceiver
{
public:
  /// Flow Control 送信用コールバック
  using SendFcCallback = std::function<Status(const CanFrame & fc_frame)>;

  /// @param send_fc  Flow Control フレーム送信用コールバック
  explicit BulkReceiver(SendFcCallback send_fc);

  /// First Frame を処理する
  /// @param eid       Extended ID (送信元/チャンネル情報)
  /// @param payload   ペイロード (type, payload_type, total_length, data...)
  /// @param len       ペイロード長
  /// @return 処理結果
  Status on_first_frame(const ExtendedId & eid, const uint8_t * payload, uint8_t len);

  /// Consecutive Frame を処理する
  /// @param eid       Extended ID
  /// @param payload   ペイロード (type, sequence, data...)
  /// @param len       ペイロード長
  /// @return 処理結果
  Status on_consecutive_frame(const ExtendedId & eid, const uint8_t * payload, uint8_t len);

  /// 現在の受信状態を取得
  BulkRxState state() const { return state_; }

  /// 受信完了したペイロードタイプを取得
  BulkPayloadType payload_type() const { return payload_type_; }

  /// 受信完了したデータを取得
  const std::vector<uint8_t> & data() const { return buffer_; }

  /// セッションをリセット
  void reset();

private:
  SendFcCallback send_fc_;
  BulkRxState state_ = BulkRxState::IDLE;
  BulkPayloadType payload_type_ = BulkPayloadType::DESCRIPTOR;
  uint32_t total_length_ = 0;
  uint8_t expected_seq_ = 1;
  std::vector<uint8_t> buffer_;
};

// ════════════════════════════════════════════════════════════════
// BULK 送信ステートマシン
// ════════════════════════════════════════════════════════════════

/// BULK 送信状態
enum class BulkTxState {
  IDLE,
  SENDING,
  WAITING_FC,
  COMPLETE,
  ERROR,
};

/// BULK 送信セッション
class BulkSender
{
public:
  /// フレーム送信用コールバック
  using SendFrameCallback = std::function<Status(const CanFrame & frame)>;

  /// @param send_frame フレーム送信用コールバック
  explicit BulkSender(SendFrameCallback send_frame);

  /// BULK 転送を開始する
  /// @param eid           送信先 Extended ID テンプレート (FC=BULK, channel を ctx に設定)
  /// @param payload_type  ペイロード種別
  /// @param data          送信データ
  /// @param len           データ長
  /// @return 処理結果
  Status start(
    const ExtendedId & eid, BulkPayloadType payload_type, const uint8_t * data, size_t len);

  /// Flow Control を受信した際の処理
  /// @param payload  FC ペイロード
  /// @param len      ペイロード長
  /// @return 処理結果
  Status on_flow_control(const uint8_t * payload, uint8_t len);

  /// 次のフレームを送信する (送信可能ならば)
  /// @return 送信完了時 true
  bool send_next();

  /// 現在の送信状態
  BulkTxState state() const { return state_; }

  /// セッションをリセット
  void reset();

private:
  SendFrameCallback send_frame_;
  BulkTxState state_ = BulkTxState::IDLE;
  ExtendedId eid_;
  std::vector<uint8_t> data_;
  size_t offset_ = 0;
  uint8_t sequence_ = 1;
  uint8_t block_size_ = 0;  // 0 = 無制限
  uint16_t st_min_us_ = 0;  // 最小分離時間
  uint8_t blocks_sent_ = 0;
};

// ════════════════════════════════════════════════════════════════
// BULK チャンネルマネージャ
// ════════════════════════════════════════════════════════════════

/// 送信チャンネル割り当て管理
class BulkChannelManager
{
public:
  /// 未使用のチャンネルを割り当てる
  /// @return 割り当てられたチャンネル番号 (0–30)、空きがない場合は nullopt
  std::optional<uint8_t> allocate();

  /// チャンネルを解放する
  void release(uint8_t channel);

  /// 全チャンネルを解放
  void reset();

private:
  uint32_t used_mask_ = 0;  // bit 0–30 で使用状況を管理
};

}  // namespace protocan
