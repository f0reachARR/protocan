#include <gtest/gtest.h>

#include "protocan/schema_hash.hpp"

using namespace protocan;

TEST(SchemaHash, Fnv1aEmptyInput)
{
  // FNV-1a of empty string should be the offset basis
  uint32_t hash = fnv1a_32(nullptr, 0);
  EXPECT_EQ(hash, 0x811C9DC5u);
}

TEST(SchemaHash, Fnv1aKnownValue)
{
  // FNV-1a("foobar") の既知の値
  const uint8_t data[] = {'f', 'o', 'o', 'b', 'a', 'r'};
  uint32_t hash = fnv1a_32(data, sizeof(data));
  EXPECT_EQ(hash, 0xBF9CF968u);
}

TEST(SchemaHash, Fnv1aSingleByte)
{
  // FNV-1a of single byte 0x00
  const uint8_t data[] = {0x00};
  uint32_t hash = fnv1a_32(data, 1);
  // hash = (0x811C9DC5 ^ 0x00) * 0x01000193
  uint32_t expected = 0x811C9DC5u * 0x01000193u;
  EXPECT_EQ(hash, expected);
}

TEST(SchemaHash, DeterministicSameInput)
{
  const uint8_t data[] = {0x0A, 0x14, 0x62, 0x6C, 0x64};
  uint32_t hash1 = fnv1a_32(data, sizeof(data));
  uint32_t hash2 = fnv1a_32(data, sizeof(data));
  EXPECT_EQ(hash1, hash2);
}

TEST(SchemaHash, ComputeSchemaHashSameAsFnv1a)
{
  const uint8_t data[] = {0x10, 0x20, 0x30};
  uint32_t h1 = fnv1a_32(data, sizeof(data));
  uint32_t h2 = compute_schema_hash(data, sizeof(data));
  EXPECT_EQ(h1, h2);
}
