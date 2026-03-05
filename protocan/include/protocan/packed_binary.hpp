#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <variant>

namespace protocan
{

/// FieldType から CAN バイナリサイズを取得
/// descriptor.proto の FieldType enum に対応 (spec §2.4)
uint8_t field_type_size(uint8_t field_type);

/// Packed Binary の値型 (デコード結果)
using FieldValue = std::variant<
  bool,      // BOOL
  uint8_t,   // UINT8
  int8_t,    // INT8
  uint16_t,  // UINT16
  int16_t,   // INT16
  uint32_t,  // UINT32
  int32_t,   // INT32
  float,     // FLOAT
  double,    // DOUBLE
  uint64_t,  // UINT64
  int64_t    // INT64
  >;

/// Packed Binary から指定 FieldType のフィールドをデコード
/// @param buf      バッファ先頭ポインタ
/// @param offset   バイトオフセット
/// @param field_type FieldType enum 値 (0–10)
/// @param size     バイトサイズ
/// @return デコードされた値
FieldValue decode_field(const uint8_t * buf, size_t offset, uint8_t field_type, uint8_t size);

/// Packed Binary に指定 FieldType のフィールドをエンコード
/// @param buf      バッファ先頭ポインタ
/// @param offset   バイトオフセット
/// @param field_type FieldType enum 値 (0–10)
/// @param value    エンコードする値
/// @param size     バイトサイズ
void encode_field(
  uint8_t * buf, size_t offset, uint8_t field_type, const FieldValue & value, uint8_t size);

}  // namespace protocan
