#include "protocan/bulk_transfer.hpp"

#include <algorithm>
#include <cstring>

namespace protocan
{

// ════════════════════════════════════════════════════════════════
// BulkReceiver
// ════════════════════════════════════════════════════════════════

BulkReceiver::BulkReceiver(SendFcCallback send_fc) : send_fc_(std::move(send_fc)) {}

Status BulkReceiver::on_first_frame(const ExtendedId & eid, const uint8_t * payload, uint8_t len)
{
  if (len < 6) {
    state_ = BulkRxState::ERROR;
    return Status::INVALID_ARGUMENT;
  }

  // [0] type (should be 0 = First Frame)
  // [1] payload_type
  // [2:5] total_length (LE u32)
  // [6..] data

  payload_type_ = static_cast<BulkPayloadType>(payload[1]);
  total_length_ = static_cast<uint32_t>(payload[2]) | (static_cast<uint32_t>(payload[3]) << 8) |
                  (static_cast<uint32_t>(payload[4]) << 16) |
                  (static_cast<uint32_t>(payload[5]) << 24);

  buffer_.clear();
  buffer_.reserve(total_length_);

  // First Frame のデータ部 (offset 6 以降)
  size_t data_len = (len > 6) ? (len - 6) : 0;
  if (data_len > 0) {
    buffer_.insert(buffer_.end(), payload + 6, payload + 6 + data_len);
  }

  expected_seq_ = 1;

  if (buffer_.size() >= total_length_) {
    state_ = BulkRxState::COMPLETE;
    buffer_.resize(total_length_);
  } else {
    state_ = BulkRxState::RECEIVING;

    // Flow Control を送信: Continue, block_size=0 (無制限), st_min=0
    if (send_fc_) {
      // FC フレームの CAN ID は受信側→送信側 (src/dst を反転)
      ExtendedId fc_eid;
      fc_eid.function_code = FunctionCode::BULK;
      fc_eid.src_dev = eid.dst_dev;
      fc_eid.src_node = eid.dst_node;
      fc_eid.dst_dev = eid.src_dev;
      fc_eid.dst_node = eid.src_node;
      fc_eid.context = eid.context;  // 同一チャンネル

      uint8_t fc_payload[5] = {};
      fc_payload[0] = static_cast<uint8_t>(BulkFrameType::FLOW_CONTROL);
      fc_payload[1] = static_cast<uint8_t>(BulkFcFlag::CONTINUE);
      fc_payload[2] = 0;  // block_size = 0 (無制限)
      fc_payload[3] = 0;  // st_min_us LE low
      fc_payload[4] = 0;  // st_min_us LE high

      CanFrame fc_frame = make_extended_frame(fc_eid, fc_payload, 5);
      send_fc_(fc_frame);
    }
  }

  return Status::OK;
}

Status BulkReceiver::on_consecutive_frame(
  const ExtendedId & eid, const uint8_t * payload, uint8_t len)
{
  if (state_ != BulkRxState::RECEIVING) {
    return Status::ERROR;
  }

  if (len < 2) {
    state_ = BulkRxState::ERROR;
    return Status::INVALID_ARGUMENT;
  }

  // [0] type (should be 1 = Consecutive Frame)
  // [1] sequence
  // [2..] data

  uint8_t sequence = payload[1];
  if (sequence != expected_seq_) {
    state_ = BulkRxState::ERROR;
    return Status::ERROR;
  }
  expected_seq_ = static_cast<uint8_t>((expected_seq_ + 1) & 0xFF);  // ラップ around at 256→0

  size_t data_len = (len > 2) ? (len - 2) : 0;
  size_t remaining = total_length_ - buffer_.size();
  size_t to_copy = std::min(data_len, remaining);

  if (to_copy > 0) {
    buffer_.insert(buffer_.end(), payload + 2, payload + 2 + to_copy);
  }

  if (buffer_.size() >= total_length_) {
    state_ = BulkRxState::COMPLETE;
    buffer_.resize(total_length_);
  }

  return Status::OK;
}

void BulkReceiver::reset()
{
  state_ = BulkRxState::IDLE;
  payload_type_ = BulkPayloadType::DESCRIPTOR;
  total_length_ = 0;
  expected_seq_ = 1;
  buffer_.clear();
}

// ════════════════════════════════════════════════════════════════
// BulkSender
// ════════════════════════════════════════════════════════════════

BulkSender::BulkSender(SendFrameCallback send_frame) : send_frame_(std::move(send_frame)) {}

Status BulkSender::start(
  const ExtendedId & eid, BulkPayloadType payload_type, const uint8_t * data, size_t len)
{
  eid_ = eid;
  data_.assign(data, data + len);
  offset_ = 0;
  sequence_ = 1;
  blocks_sent_ = 0;
  state_ = BulkTxState::SENDING;

  // First Frame を送信
  // [0] type=0 (First Frame)
  // [1] payload_type
  // [2:5] total_length (LE u32)
  // [6..] data (最大 58 bytes)
  uint8_t frame_payload[64] = {};
  frame_payload[0] = static_cast<uint8_t>(BulkFrameType::FIRST_FRAME);
  frame_payload[1] = static_cast<uint8_t>(payload_type);
  frame_payload[2] = static_cast<uint8_t>(len);
  frame_payload[3] = static_cast<uint8_t>(len >> 8);
  frame_payload[4] = static_cast<uint8_t>(len >> 16);
  frame_payload[5] = static_cast<uint8_t>(len >> 24);

  size_t first_data_len = std::min(len, static_cast<size_t>(58));  // 64 - 6 header bytes
  if (first_data_len > 0) {
    std::memcpy(frame_payload + 6, data, first_data_len);
  }
  offset_ = first_data_len;

  uint8_t frame_len = static_cast<uint8_t>(6 + first_data_len);
  CanFrame frame = make_extended_frame(eid_, frame_payload, frame_len);

  Status s = send_frame_(frame);
  if (s != Status::OK) {
    state_ = BulkTxState::ERROR;
    return s;
  }

  if (offset_ >= data_.size()) {
    state_ = BulkTxState::COMPLETE;
  } else {
    state_ = BulkTxState::WAITING_FC;
  }

  return Status::OK;
}

Status BulkSender::on_flow_control(const uint8_t * payload, uint8_t len)
{
  if (state_ != BulkTxState::WAITING_FC) {
    return Status::ERROR;
  }

  if (len < 3) {
    state_ = BulkTxState::ERROR;
    return Status::INVALID_ARGUMENT;
  }

  // [0] type = 2 (Flow Control)
  // [1] fc_flag
  // [2] block_size
  // [3:4] st_min_us (optional)
  auto fc_flag = static_cast<BulkFcFlag>(payload[1]);
  if (fc_flag == BulkFcFlag::ABORT) {
    state_ = BulkTxState::ERROR;
    return Status::ERROR;
  }
  if (fc_flag == BulkFcFlag::WAIT) {
    // 待機状態を維持
    return Status::OK;
  }

  block_size_ = payload[2];
  if (len >= 5) {
    st_min_us_ = static_cast<uint16_t>(payload[3]) | (static_cast<uint16_t>(payload[4]) << 8);
  }

  blocks_sent_ = 0;
  state_ = BulkTxState::SENDING;

  return Status::OK;
}

bool BulkSender::send_next()
{
  if (state_ != BulkTxState::SENDING) {
    return false;
  }

  if (offset_ >= data_.size()) {
    state_ = BulkTxState::COMPLETE;
    return true;
  }

  // Consecutive Frame
  // [0] type=1 (Consecutive Frame)
  // [1] sequence
  // [2..] data (最大 62 bytes)
  uint8_t frame_payload[64] = {};
  frame_payload[0] = static_cast<uint8_t>(BulkFrameType::CONSECUTIVE_FRAME);
  frame_payload[1] = sequence_;
  sequence_ = static_cast<uint8_t>((sequence_ + 1) & 0xFF);  // ラップ

  size_t remaining = data_.size() - offset_;
  size_t chunk_len = std::min(remaining, static_cast<size_t>(62));  // 64 - 2 header bytes
  std::memcpy(frame_payload + 2, data_.data() + offset_, chunk_len);
  offset_ += chunk_len;

  uint8_t frame_len = static_cast<uint8_t>(2 + chunk_len);
  CanFrame frame = make_extended_frame(eid_, frame_payload, frame_len);

  Status s = send_frame_(frame);
  if (s != Status::OK) {
    state_ = BulkTxState::ERROR;
    return false;
  }

  blocks_sent_++;

  if (offset_ >= data_.size()) {
    state_ = BulkTxState::COMPLETE;
    return true;
  }

  // block_size が0でない場合、指定ブロック数送信後に FC 待ち
  if (block_size_ > 0 && blocks_sent_ >= block_size_) {
    state_ = BulkTxState::WAITING_FC;
    blocks_sent_ = 0;
  }

  return false;
}

void BulkSender::reset()
{
  state_ = BulkTxState::IDLE;
  data_.clear();
  offset_ = 0;
  sequence_ = 1;
  block_size_ = 0;
  st_min_us_ = 0;
  blocks_sent_ = 0;
}

// ════════════════════════════════════════════════════════════════
// BulkChannelManager
// ════════════════════════════════════════════════════════════════

std::optional<uint8_t> BulkChannelManager::allocate()
{
  for (uint8_t ch = 0; ch <= kBulkMaxChannel; ++ch) {
    if (!(used_mask_ & (1u << ch))) {
      used_mask_ |= (1u << ch);
      return ch;
    }
  }
  return std::nullopt;
}

void BulkChannelManager::release(uint8_t channel)
{
  if (channel <= kBulkMaxChannel) {
    used_mask_ &= ~(1u << channel);
  }
}

void BulkChannelManager::reset() { used_mask_ = 0; }

}  // namespace protocan
