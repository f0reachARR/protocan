#include "protocan/schema_hash.hpp"

namespace protocan
{

// FNV-1a (32-bit) 定数
static constexpr uint32_t kFnv1aOffsetBasis = 0x811C9DC5u;
static constexpr uint32_t kFnv1aPrime = 0x01000193u;

uint32_t fnv1a_32(const uint8_t * data, size_t len)
{
  uint32_t hash = kFnv1aOffsetBasis;
  for (size_t i = 0; i < len; ++i) {
    hash ^= static_cast<uint32_t>(data[i]);
    hash *= kFnv1aPrime;
  }
  return hash;
}

uint32_t compute_schema_hash(const uint8_t * serialized_descriptor, size_t len)
{
  return fnv1a_32(serialized_descriptor, len);
}

}  // namespace protocan
