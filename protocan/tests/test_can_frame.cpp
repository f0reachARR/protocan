#include <gtest/gtest.h>

#include "protocan/can_frame.hpp"

using namespace protocan;

TEST(CanFrame, EncodeDecodeRoundtrip)
{
  ExtendedId eid;
  eid.function_code = FunctionCode::PARAM;
  eid.src_dev = 0;  // master
  eid.src_node = 0;
  eid.dst_dev = 5;
  eid.dst_node = 12;
  eid.context = 0x02;  // WRITE

  uint32_t raw = encode_extended_id(eid);
  ExtendedId decoded = decode_extended_id(raw);

  EXPECT_EQ(decoded.function_code, FunctionCode::PARAM);
  EXPECT_EQ(decoded.src_dev, 0);
  EXPECT_EQ(decoded.src_node, 0);
  EXPECT_EQ(decoded.dst_dev, 5);
  EXPECT_EQ(decoded.dst_node, 12);
  EXPECT_EQ(decoded.context, 0x02);
}

TEST(CanFrame, AllBitsMaxValues)
{
  ExtendedId eid;
  eid.function_code = FunctionCode::SERVICE;  // 0x8 → 4 bits
  eid.src_dev = 0x0F;                         // 4 bits max
  eid.src_node = 0x3F;                        // 6 bits max
  eid.dst_dev = 0x0F;                         // 4 bits max
  eid.dst_node = 0x3F;                        // 6 bits max
  eid.context = 0x1F;                         // 5 bits max

  uint32_t raw = encode_extended_id(eid);
  ExtendedId decoded = decode_extended_id(raw);

  EXPECT_EQ(decoded.function_code, FunctionCode::SERVICE);
  EXPECT_EQ(decoded.src_dev, 0x0F);
  EXPECT_EQ(decoded.src_node, 0x3F);
  EXPECT_EQ(decoded.dst_dev, 0x0F);
  EXPECT_EQ(decoded.dst_node, 0x3F);
  EXPECT_EQ(decoded.context, 0x1F);
}

TEST(CanFrame, NmtHeartbeatId)
{
  // spec §5.1: HEARTBEAT CAN ID
  // NMT, src_dev=device(3), src_node=0, dst_dev=0xF(broadcast), dst_node=0x3F(broadcast), ctx=0
  ExtendedId eid;
  eid.function_code = FunctionCode::NMT;
  eid.src_dev = 3;
  eid.src_node = 0;
  eid.dst_dev = kBroadcastDeviceId;
  eid.dst_node = kBroadcastNodeId;
  eid.context = 0;

  uint32_t raw = encode_extended_id(eid);
  ExtendedId decoded = decode_extended_id(raw);

  EXPECT_EQ(decoded.function_code, FunctionCode::NMT);
  EXPECT_EQ(decoded.src_dev, 3);
  EXPECT_EQ(decoded.src_node, 0);
  EXPECT_EQ(decoded.dst_dev, kBroadcastDeviceId);
  EXPECT_EQ(decoded.dst_node, kBroadcastNodeId);
  EXPECT_EQ(decoded.context, 0);
}

TEST(CanFrame, MakeExtendedFrame)
{
  ExtendedId eid;
  eid.function_code = FunctionCode::DISC;
  eid.src_dev = 0;
  eid.src_node = 0;
  eid.dst_dev = 2;
  eid.dst_node = 5;
  eid.context = 0;

  uint8_t payload[] = {0x01, 0x02};
  CanFrame frame = make_extended_frame(eid, payload, 2);

  EXPECT_TRUE(frame.is_extended);
  EXPECT_TRUE(frame.is_fd);
  EXPECT_EQ(frame.dlc, 2);
  EXPECT_EQ(frame.data[0], 0x01);
  EXPECT_EQ(frame.data[1], 0x02);
}

TEST(CanFrame, MakeStandardFrame)
{
  uint8_t payload[] = {0xAA, 0xBB, 0xCC};
  CanFrame frame = make_standard_frame(0x181, payload, 3);

  EXPECT_FALSE(frame.is_extended);
  EXPECT_TRUE(frame.is_fd);
  EXPECT_EQ(frame.id, 0x181u);
  EXPECT_EQ(frame.dlc, 3);
  EXPECT_EQ(frame.data[0], 0xAA);
  EXPECT_EQ(frame.data[1], 0xBB);
  EXPECT_EQ(frame.data[2], 0xCC);
}

TEST(CanFrame, IsManagementVsPdo)
{
  CanFrame ext_frame;
  ext_frame.is_extended = true;
  EXPECT_TRUE(is_management_frame(ext_frame));
  EXPECT_FALSE(is_pdo_frame(ext_frame));

  CanFrame std_frame;
  std_frame.is_extended = false;
  EXPECT_FALSE(is_management_frame(std_frame));
  EXPECT_TRUE(is_pdo_frame(std_frame));
}
