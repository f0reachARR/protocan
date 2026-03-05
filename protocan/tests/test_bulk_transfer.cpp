#include <gtest/gtest.h>

#include <cstring>

#include "protocan/bulk_transfer.hpp"

using namespace protocan;

class BulkTransferTest : public ::testing::Test
{
protected:
  std::vector<CanFrame> sent_frames;

  Status mock_send(const CanFrame & frame)
  {
    sent_frames.push_back(frame);
    return Status::OK;
  }

  ExtendedId make_bulk_eid(
    uint8_t src_dev, uint8_t src_node, uint8_t dst_dev, uint8_t dst_node, uint8_t channel)
  {
    ExtendedId eid;
    eid.function_code = FunctionCode::BULK;
    eid.src_dev = src_dev;
    eid.src_node = src_node;
    eid.dst_dev = dst_dev;
    eid.dst_node = dst_node;
    eid.context = channel;
    return eid;
  }
};

TEST_F(BulkTransferTest, ReceiverSmallPayload)
{
  // 小さなデータ (First Frame のみで完結)
  auto send_fc = [this](const CanFrame & f) -> Status { return mock_send(f); };
  BulkReceiver rx(send_fc);

  ExtendedId eid = make_bulk_eid(3, 0, 0, 0, 0);

  // First Frame: type=0, payload_type=0(Descriptor), total_length=4, data=[0xAA,0xBB,0xCC,0xDD]
  uint8_t ff[10] = {};
  ff[0] = 0;  // FIRST_FRAME
  ff[1] = 0;  // DESCRIPTOR
  ff[2] = 4;
  ff[3] = 0;
  ff[4] = 0;
  ff[5] = 0;  // total_length = 4
  ff[6] = 0xAA;
  ff[7] = 0xBB;
  ff[8] = 0xCC;
  ff[9] = 0xDD;

  rx.on_first_frame(eid, ff, 10);

  EXPECT_EQ(rx.state(), BulkRxState::COMPLETE);
  EXPECT_EQ(rx.payload_type(), BulkPayloadType::DESCRIPTOR);
  ASSERT_EQ(rx.data().size(), 4u);
  EXPECT_EQ(rx.data()[0], 0xAA);
  EXPECT_EQ(rx.data()[3], 0xDD);
}

TEST_F(BulkTransferTest, ReceiverMultiFrame)
{
  // 複数フレームにまたがるデータ
  auto send_fc = [this](const CanFrame & f) -> Status { return mock_send(f); };
  BulkReceiver rx(send_fc);

  ExtendedId eid = make_bulk_eid(3, 0, 0, 0, 1);

  // total_length = 70 (First Frame data: 58 bytes, CF data: 12 bytes)
  uint8_t ff[64] = {};
  ff[0] = 0;  // FIRST_FRAME
  ff[1] = 0;  // DESCRIPTOR
  ff[2] = 70;
  ff[3] = 0;
  ff[4] = 0;
  ff[5] = 0;  // total_length = 70
  // data [6..63] = 58 bytes of 0x11
  std::memset(ff + 6, 0x11, 58);

  rx.on_first_frame(eid, ff, 64);
  EXPECT_EQ(rx.state(), BulkRxState::RECEIVING);
  // FC should have been sent
  EXPECT_EQ(sent_frames.size(), 1u);

  // Consecutive Frame: remaining 12 bytes
  uint8_t cf[14] = {};
  cf[0] = 1;  // CONSECUTIVE_FRAME
  cf[1] = 1;  // sequence = 1
  std::memset(cf + 2, 0x22, 12);

  rx.on_consecutive_frame(eid, cf, 14);
  EXPECT_EQ(rx.state(), BulkRxState::COMPLETE);
  ASSERT_EQ(rx.data().size(), 70u);
  EXPECT_EQ(rx.data()[0], 0x11);   // first frame data
  EXPECT_EQ(rx.data()[57], 0x11);  // last byte of first frame data
  EXPECT_EQ(rx.data()[58], 0x22);  // first byte of CF data
  EXPECT_EQ(rx.data()[69], 0x22);  // last byte
}

TEST_F(BulkTransferTest, SenderSmallPayload)
{
  auto send_fn = [this](const CanFrame & f) -> Status { return mock_send(f); };
  BulkSender tx(send_fn);

  ExtendedId eid = make_bulk_eid(0, 0, 3, 5, 0);

  uint8_t data[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
  tx.start(eid, BulkPayloadType::DESCRIPTOR, data, 10);

  // Small enough to fit in First Frame (58 bytes data capacity)
  EXPECT_EQ(tx.state(), BulkTxState::COMPLETE);
  EXPECT_EQ(sent_frames.size(), 1u);
}

TEST_F(BulkTransferTest, ChannelManagerAllocRelease)
{
  BulkChannelManager mgr;

  // 31 チャンネル (0-30) を全て割り当て可能
  for (uint8_t i = 0; i <= 30; ++i) {
    auto ch = mgr.allocate();
    ASSERT_TRUE(ch.has_value());
    EXPECT_EQ(*ch, i);
  }

  // もう空きがない
  EXPECT_FALSE(mgr.allocate().has_value());

  // 解放して再割り当て
  mgr.release(5);
  auto ch = mgr.allocate();
  ASSERT_TRUE(ch.has_value());
  EXPECT_EQ(*ch, 5);
}

TEST_F(BulkTransferTest, ChannelManagerReset)
{
  BulkChannelManager mgr;

  mgr.allocate();
  mgr.allocate();
  mgr.allocate();

  mgr.reset();

  auto ch = mgr.allocate();
  ASSERT_TRUE(ch.has_value());
  EXPECT_EQ(*ch, 0);  // リセット後は 0 から
}
