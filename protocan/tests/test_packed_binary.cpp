#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "protocan/packed_binary.hpp"

using namespace protocan;

TEST(PackedBinary, Bool)
{
  uint8_t buf[1] = {};
  encode_field(buf, 0, 0 /*BOOL*/, true, 1);
  EXPECT_EQ(buf[0], 1);

  auto val = decode_field(buf, 0, 0, 1);
  EXPECT_TRUE(std::get<bool>(val));

  encode_field(buf, 0, 0, false, 1);
  EXPECT_EQ(buf[0], 0);
  val = decode_field(buf, 0, 0, 1);
  EXPECT_FALSE(std::get<bool>(val));
}

TEST(PackedBinary, Uint8)
{
  uint8_t buf[1] = {};
  encode_field(buf, 0, 1 /*UINT8*/, static_cast<uint8_t>(0xAB), 1);
  EXPECT_EQ(buf[0], 0xAB);

  auto val = decode_field(buf, 0, 1, 1);
  EXPECT_EQ(std::get<uint8_t>(val), 0xAB);
}

TEST(PackedBinary, Int8)
{
  uint8_t buf[1] = {};
  encode_field(buf, 0, 2 /*INT8*/, static_cast<int8_t>(-42), 1);

  auto val = decode_field(buf, 0, 2, 1);
  EXPECT_EQ(std::get<int8_t>(val), -42);
}

TEST(PackedBinary, Uint16LE)
{
  uint8_t buf[2] = {};
  encode_field(buf, 0, 3 /*UINT16*/, static_cast<uint16_t>(0x1234), 2);
  EXPECT_EQ(buf[0], 0x34);  // LE low byte
  EXPECT_EQ(buf[1], 0x12);  // LE high byte

  auto val = decode_field(buf, 0, 3, 2);
  EXPECT_EQ(std::get<uint16_t>(val), 0x1234);
}

TEST(PackedBinary, Int16LE)
{
  uint8_t buf[2] = {};
  encode_field(buf, 0, 4 /*INT16*/, static_cast<int16_t>(-1000), 2);

  auto val = decode_field(buf, 0, 4, 2);
  EXPECT_EQ(std::get<int16_t>(val), -1000);
}

TEST(PackedBinary, Uint32LE)
{
  uint8_t buf[4] = {};
  encode_field(buf, 0, 5 /*UINT32*/, static_cast<uint32_t>(0xDEADBEEF), 4);
  EXPECT_EQ(buf[0], 0xEF);
  EXPECT_EQ(buf[1], 0xBE);
  EXPECT_EQ(buf[2], 0xAD);
  EXPECT_EQ(buf[3], 0xDE);

  auto val = decode_field(buf, 0, 5, 4);
  EXPECT_EQ(std::get<uint32_t>(val), 0xDEADBEEFu);
}

TEST(PackedBinary, Int32LE)
{
  uint8_t buf[4] = {};
  encode_field(buf, 0, 6 /*INT32*/, static_cast<int32_t>(-123456), 4);

  auto val = decode_field(buf, 0, 6, 4);
  EXPECT_EQ(std::get<int32_t>(val), -123456);
}

TEST(PackedBinary, Float)
{
  uint8_t buf[4] = {};
  encode_field(buf, 0, 7 /*FLOAT*/, 3.14f, 4);

  auto val = decode_field(buf, 0, 7, 4);
  EXPECT_NEAR(std::get<float>(val), 3.14f, 1e-6f);
}

TEST(PackedBinary, Double)
{
  uint8_t buf[8] = {};
  encode_field(buf, 0, 8 /*DOUBLE*/, 2.718281828, 8);

  auto val = decode_field(buf, 0, 8, 8);
  EXPECT_NEAR(std::get<double>(val), 2.718281828, 1e-9);
}

TEST(PackedBinary, Uint64LE)
{
  uint8_t buf[8] = {};
  uint64_t v = 0x0102030405060708ULL;
  encode_field(buf, 0, 9 /*UINT64*/, v, 8);
  EXPECT_EQ(buf[0], 0x08);
  EXPECT_EQ(buf[7], 0x01);

  auto val = decode_field(buf, 0, 9, 8);
  EXPECT_EQ(std::get<uint64_t>(val), v);
}

TEST(PackedBinary, Int64LE)
{
  uint8_t buf[8] = {};
  encode_field(buf, 0, 10 /*INT64*/, static_cast<int64_t>(-999999999999LL), 8);

  auto val = decode_field(buf, 0, 10, 8);
  EXPECT_EQ(std::get<int64_t>(val), -999999999999LL);
}

TEST(PackedBinary, OffsetEncoding)
{
  // エンコード先のオフセットが正しく適用されるか
  uint8_t buf[16] = {};
  encode_field(buf, 4, 5 /*UINT32*/, static_cast<uint32_t>(0x12345678), 4);

  EXPECT_EQ(buf[0], 0x00);  // offset 0-3 はゼロ
  EXPECT_EQ(buf[4], 0x78);  // offset 4
  EXPECT_EQ(buf[5], 0x56);
  EXPECT_EQ(buf[6], 0x34);
  EXPECT_EQ(buf[7], 0x12);

  auto val = decode_field(buf, 4, 5, 4);
  EXPECT_EQ(std::get<uint32_t>(val), 0x12345678u);
}

TEST(PackedBinary, FieldTypeSize)
{
  EXPECT_EQ(field_type_size(0), 1);   // BOOL
  EXPECT_EQ(field_type_size(1), 1);   // UINT8
  EXPECT_EQ(field_type_size(2), 1);   // INT8
  EXPECT_EQ(field_type_size(3), 2);   // UINT16
  EXPECT_EQ(field_type_size(4), 2);   // INT16
  EXPECT_EQ(field_type_size(5), 4);   // UINT32
  EXPECT_EQ(field_type_size(6), 4);   // INT32
  EXPECT_EQ(field_type_size(7), 4);   // FLOAT
  EXPECT_EQ(field_type_size(8), 8);   // DOUBLE
  EXPECT_EQ(field_type_size(9), 8);   // UINT64
  EXPECT_EQ(field_type_size(10), 8);  // INT64
}
