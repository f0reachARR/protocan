#pragma once

#include <array>
#include <cstdint>
#include <cstring>

#include "protocan/types.hpp"

namespace protocan
{

// ════════════════════════════════════════════════════════════════
// CAN FD フレーム表現
// ════════════════════════════════════════════════════════════════

/// CAN FD フレーム
struct CanFrame
{
  uint32_t id = 0;
  bool is_extended = false;  // Extended (29-bit) or Standard (11-bit)
  bool is_fd = true;         // CAN FD フレーム
  uint8_t dlc = 0;           // Data Length Code
  std::array<uint8_t, 64> data = {};

  /// データ部分をゼロクリア
  void clear_data() { data.fill(0); }
};

// ════════════════════════════════════════════════════════════════
// Extended ID (29-bit) ビットフィールド構造体
// ════════════════════════════════════════════════════════════════

/// Extended ID のビットフィールド分解 (spec §4.1)
///
/// Bit [28:25] Function Code    (4 bit)
/// Bit [24:21] Source Device ID (4 bit)
/// Bit [20:15] Source Local Node(6 bit)
/// Bit [14:11] Dest Device ID   (4 bit)
/// Bit [10:5]  Dest Local Node  (6 bit)
/// Bit [4:0]   Context / Seq    (5 bit)
struct ExtendedId
{
  FunctionCode function_code = FunctionCode::NMT;
  uint8_t src_dev = 0;   // 4 bit: 0–15
  uint8_t src_node = 0;  // 6 bit: 0–63
  uint8_t dst_dev = 0;   // 4 bit: 0–15
  uint8_t dst_node = 0;  // 6 bit: 0–63
  uint8_t context = 0;   // 5 bit: 0–31
};

/// ExtendedId → 29-bit CAN ID にエンコード
uint32_t encode_extended_id(const ExtendedId & eid);

/// 29-bit CAN ID → ExtendedId にデコード
ExtendedId decode_extended_id(uint32_t raw_id);

// ════════════════════════════════════════════════════════════════
// CanFrame ヘルパー
// ════════════════════════════════════════════════════════════════

/// Extended ID フレームを構築
CanFrame make_extended_frame(const ExtendedId & eid, const uint8_t * payload, uint8_t len);

/// Standard ID (PDO) フレームを構築
CanFrame make_standard_frame(uint16_t pdo_id, const uint8_t * payload, uint8_t len);

/// フレームが Extended ID かどうか判定
inline bool is_management_frame(const CanFrame & frame) { return frame.is_extended; }

/// フレームが Standard ID (PDO) かどうか判定
inline bool is_pdo_frame(const CanFrame & frame) { return !frame.is_extended; }

}  // namespace protocan
