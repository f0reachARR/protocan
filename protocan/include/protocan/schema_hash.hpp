#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace protocan
{

/// FNV-1a (32-bit) ハッシュを計算 (spec §3.1)
///
/// @param data  入力バイト列
/// @param len   入力バイト列の長さ
/// @return      32-bit FNV-1a ハッシュ値
uint32_t fnv1a_32(const uint8_t * data, size_t len);

/// NodeDescriptor の schema_hash を計算 (descriptor_spec.md §4)
///
/// 手順:
///  1. schema_hash フィールドを 0 にセット
///  2. Deterministic serialization でバイト列化
///  3. FNV-1a (32-bit) を計算して返す
///
/// @param serialized_descriptor  schema_hash=0 で deterministic serialize したバイト列
/// @param len                    バイト列の長さ
/// @return                       32-bit schema hash
uint32_t compute_schema_hash(const uint8_t * serialized_descriptor, size_t len);

}  // namespace protocan
