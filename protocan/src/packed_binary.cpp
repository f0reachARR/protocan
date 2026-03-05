#include "protocan/packed_binary.hpp"

#include <stdexcept>

namespace protocan
{

// FieldType enum values (matching descriptor.proto)
static constexpr uint8_t FT_BOOL = 0;
static constexpr uint8_t FT_UINT8 = 1;
static constexpr uint8_t FT_INT8 = 2;
static constexpr uint8_t FT_UINT16 = 3;
static constexpr uint8_t FT_INT16 = 4;
static constexpr uint8_t FT_UINT32 = 5;
static constexpr uint8_t FT_INT32 = 6;
static constexpr uint8_t FT_FLOAT = 7;
static constexpr uint8_t FT_DOUBLE = 8;
static constexpr uint8_t FT_UINT64 = 9;
static constexpr uint8_t FT_INT64 = 10;

uint8_t field_type_size(uint8_t field_type)
{
  switch (field_type) {
    case FT_BOOL:
      return 1;
    case FT_UINT8:
      return 1;
    case FT_INT8:
      return 1;
    case FT_UINT16:
      return 2;
    case FT_INT16:
      return 2;
    case FT_UINT32:
      return 4;
    case FT_INT32:
      return 4;
    case FT_FLOAT:
      return 4;
    case FT_DOUBLE:
      return 8;
    case FT_UINT64:
      return 8;
    case FT_INT64:
      return 8;
    default:
      return 0;
  }
}

// ── LE ヘルパー ──

static inline uint16_t read_le16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

static inline uint32_t read_le32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

static inline uint64_t read_le64(const uint8_t * p)
{
  return static_cast<uint64_t>(p[0]) | (static_cast<uint64_t>(p[1]) << 8) |
         (static_cast<uint64_t>(p[2]) << 16) | (static_cast<uint64_t>(p[3]) << 24) |
         (static_cast<uint64_t>(p[4]) << 32) | (static_cast<uint64_t>(p[5]) << 40) |
         (static_cast<uint64_t>(p[6]) << 48) | (static_cast<uint64_t>(p[7]) << 56);
}

static inline void write_le16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}

static inline void write_le32(uint8_t * p, uint32_t v)
{
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}

static inline void write_le64(uint8_t * p, uint64_t v)
{
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
  p[4] = static_cast<uint8_t>(v >> 32);
  p[5] = static_cast<uint8_t>(v >> 40);
  p[6] = static_cast<uint8_t>(v >> 48);
  p[7] = static_cast<uint8_t>(v >> 56);
}

FieldValue decode_field(const uint8_t * buf, size_t offset, uint8_t field_type, uint8_t size)
{
  const uint8_t * p = buf + offset;

  switch (field_type) {
    case FT_BOOL:
      return static_cast<bool>(p[0] != 0);
    case FT_UINT8:
      return p[0];
    case FT_INT8:
      return static_cast<int8_t>(p[0]);
    case FT_UINT16:
      return read_le16(p);
    case FT_INT16:
      return static_cast<int16_t>(read_le16(p));
    case FT_UINT32:
      return read_le32(p);
    case FT_INT32:
      return static_cast<int32_t>(read_le32(p));
    case FT_FLOAT: {
      uint32_t raw = read_le32(p);
      float f;
      std::memcpy(&f, &raw, sizeof(f));
      return f;
    }
    case FT_DOUBLE: {
      uint64_t raw = read_le64(p);
      double d;
      std::memcpy(&d, &raw, sizeof(d));
      return d;
    }
    case FT_UINT64:
      return read_le64(p);
    case FT_INT64:
      return static_cast<int64_t>(read_le64(p));
    default:
      return static_cast<uint8_t>(0);
  }
}

void encode_field(
  uint8_t * buf, size_t offset, uint8_t field_type, const FieldValue & value, uint8_t size)
{
  uint8_t * p = buf + offset;

  switch (field_type) {
    case FT_BOOL:
      p[0] = std::get<bool>(value) ? 1 : 0;
      break;
    case FT_UINT8:
      p[0] = std::get<uint8_t>(value);
      break;
    case FT_INT8:
      p[0] = static_cast<uint8_t>(std::get<int8_t>(value));
      break;
    case FT_UINT16:
      write_le16(p, std::get<uint16_t>(value));
      break;
    case FT_INT16:
      write_le16(p, static_cast<uint16_t>(std::get<int16_t>(value)));
      break;
    case FT_UINT32:
      write_le32(p, std::get<uint32_t>(value));
      break;
    case FT_INT32:
      write_le32(p, static_cast<uint32_t>(std::get<int32_t>(value)));
      break;
    case FT_FLOAT: {
      float f = std::get<float>(value);
      uint32_t raw;
      std::memcpy(&raw, &f, sizeof(raw));
      write_le32(p, raw);
      break;
    }
    case FT_DOUBLE: {
      double d = std::get<double>(value);
      uint64_t raw;
      std::memcpy(&raw, &d, sizeof(raw));
      write_le64(p, raw);
      break;
    }
    case FT_UINT64:
      write_le64(p, std::get<uint64_t>(value));
      break;
    case FT_INT64:
      write_le64(p, static_cast<uint64_t>(std::get<int64_t>(value)));
      break;
    default:
      break;
  }
}

}  // namespace protocan
