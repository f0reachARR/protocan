#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "protocan/types.hpp"

// Forward declarations for Protobuf generated types
namespace protocan
{
class NodeDescriptor;
class MessageDescriptor;
}  // namespace protocan

namespace protocan
{

/// パース済みフィールド情報
struct ParsedField
{
  std::string name;
  uint8_t type;  // FieldType enum
  uint32_t offset;
  uint32_t size;
  std::string ros2_field;
};

/// パース済みメッセージ情報
struct ParsedMessage
{
  std::string ros2_msg_type;
  uint32_t payload_size;
  std::vector<ParsedField> fields;
};

/// パース済みトピック情報
struct ParsedTopic
{
  uint32_t index;
  std::string name;
  bool is_tx;
  bool periodic;
  uint32_t priority;
  ParsedMessage message;
};

/// パース済みサービス情報
struct ParsedService
{
  uint32_t index;
  std::string name;
  std::string ros2_srv_type;
  ParsedMessage request;
  ParsedMessage response;
};

/// パース済みパラメータ情報
struct ParsedParam
{
  uint32_t index;
  std::string name;
  uint8_t type;  // FieldType enum
  bool read_only;
};

/// パース済みノードディスクリプタ
struct ParsedDescriptor
{
  uint32_t schema_hash;
  std::string node_type_name;
  std::string ros2_namespace;
  std::vector<ParsedTopic> topics;
  std::vector<ParsedService> services;
  std::vector<ParsedParam> params;
};

/// Descriptor Blob (Protobuf バイナリ) をパースする
///
/// @param blob  Protobuf シリアライズされた NodeDescriptor バイナリ
/// @param len   バイナリの長さ
/// @param out   パース結果の格納先
/// @return      パース成功時 true
bool parse_descriptor(const uint8_t * blob, size_t len, ParsedDescriptor & out);

/// Descriptor Blob を検証する
///
/// @param desc  パース済みディスクリプタ
/// @return      妥当な場合 true
bool validate_descriptor(const ParsedDescriptor & desc);

/// デバイス間ルーティングのペイロード互換性を検証 (descriptor_spec.md §3.5)
///
/// payload_size, フィールド数, 各フィールドの type/offset/size が全て一致するか検証する。
/// フィールド名や ros2_field の一致は要求しない。
///
/// @return 互換性がある場合 true
bool check_payload_compatibility(const ParsedMessage & a, const ParsedMessage & b);

}  // namespace protocan
