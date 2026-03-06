#include <gtest/gtest.h>

#include "protocan_device/node_base.hpp"

using namespace protocan;
using namespace protocan::device;

// ── テスト用具体的 NodeBase サブクラス ──

static const uint8_t kTestBlob[] = {0xAA, 0xBB, 0xCC};

class TestNode : public NodeBase
{
public:
  TestNode(uint8_t id)
  : NodeBase(id, "test_node", kTestBlob, sizeof(kTestBlob), 0xDEADBEEF)
  {
  }

  void on_pdo_rx(uint16_t, const uint8_t *, uint8_t) override {}
  uint8_t fill_pdo_tx(uint16_t, uint8_t *, uint8_t) override { return 0; }
  Status on_param_read(uint8_t, uint8_t *, uint8_t &) override { return Status::NOT_FOUND; }
  Status on_param_write(uint8_t, const uint8_t *, uint8_t) override { return Status::NOT_FOUND; }
  Status on_service_req(uint8_t, const uint8_t *, uint8_t, uint8_t *, uint8_t &) override
  {
    return Status::NOT_FOUND;
  }
};

// ════════════════════════════════════════════════════════════════
// NodeBase 基本プロパティ
// ════════════════════════════════════════════════════════════════

TEST(NodeBaseTest, BasicProperties)
{
  TestNode node(5);
  EXPECT_EQ(node.local_id(), 5u);
  EXPECT_STREQ(node.instance_name(), "test_node");
  EXPECT_EQ(node.schema_hash(), 0xDEADBEEFu);
  EXPECT_EQ(node.descriptor_blob(), kTestBlob);
  EXPECT_EQ(node.descriptor_blob_size(), sizeof(kTestBlob));
  EXPECT_EQ(node.pdo_rx_count(), 0u);
  EXPECT_EQ(node.pdo_tx_count(), 0u);
}

// ════════════════════════════════════════════════════════════════
// PDO RX 追加・上限
// ════════════════════════════════════════════════════════════════

TEST(NodeBaseTest, AddPdoRx)
{
  TestNode node(1);

  PdoRxEntry e{0x100, 0, 0, 0, 4};
  EXPECT_EQ(node.add_pdo_rx(e), Status::OK);
  EXPECT_EQ(node.pdo_rx_count(), 1u);
  EXPECT_EQ(node.pdo_rx_at(0).pdo_id, 0x100u);
  EXPECT_EQ(node.pdo_rx_at(0).topic_index, 0u);
  EXPECT_EQ(node.pdo_rx_at(0).field_index, 0u);
  EXPECT_EQ(node.pdo_rx_at(0).offset, 0u);
  EXPECT_EQ(node.pdo_rx_at(0).size, 4u);
}

TEST(NodeBaseTest, AddPdoRxLimit)
{
  TestNode node(1);

  for (uint8_t i = 0; i < kMaxPdoPerNode; ++i) {
    PdoRxEntry e{static_cast<uint16_t>(0x100 + i), i, 0, 0, 4};
    EXPECT_EQ(node.add_pdo_rx(e), Status::OK);
  }
  EXPECT_EQ(node.pdo_rx_count(), kMaxPdoPerNode);

  // 上限超え
  PdoRxEntry overflow{0x200, 0, 0, 0, 4};
  EXPECT_EQ(node.add_pdo_rx(overflow), Status::NO_RESOURCE);
  EXPECT_EQ(node.pdo_rx_count(), kMaxPdoPerNode);
}

// ════════════════════════════════════════════════════════════════
// PDO TX 追加・上限
// ════════════════════════════════════════════════════════════════

TEST(NodeBaseTest, AddPdoTx)
{
  TestNode node(1);

  PdoTxEntry e{0x200, 1, 0, 0, 0, 100, 0};
  EXPECT_EQ(node.add_pdo_tx(e), Status::OK);
  EXPECT_EQ(node.pdo_tx_count(), 1u);
  EXPECT_EQ(node.pdo_tx_at(0).pdo_id, 0x200u);
  EXPECT_EQ(node.pdo_tx_at(0).topic_index, 1u);
  EXPECT_EQ(node.pdo_tx_at(0).field_index, 0u);
  EXPECT_EQ(node.pdo_tx_at(0).period_ms, 100u);
}

TEST(NodeBaseTest, AddPdoTxLimit)
{
  TestNode node(1);

  for (uint8_t i = 0; i < kMaxPdoPerNode; ++i) {
    PdoTxEntry e{static_cast<uint16_t>(0x200 + i), i, 0, 0, 0, 100, 0};
    EXPECT_EQ(node.add_pdo_tx(e), Status::OK);
  }
  EXPECT_EQ(node.pdo_tx_count(), kMaxPdoPerNode);

  PdoTxEntry overflow{0x300, 0, 0, 0, 0, 100, 0};
  EXPECT_EQ(node.add_pdo_tx(overflow), Status::NO_RESOURCE);
  EXPECT_EQ(node.pdo_tx_count(), kMaxPdoPerNode);
}

// ════════════════════════════════════════════════════════════════
// clear_pdo: 指定 PDO ID を削除
// ════════════════════════════════════════════════════════════════

TEST(NodeBaseTest, ClearPdo)
{
  TestNode node(1);

  node.add_pdo_rx({0x100, 0, 0, 0, 4});
  node.add_pdo_rx({0x101, 1, 0, 4, 4});
  node.add_pdo_rx({0x100, 2, 0, 8, 4});  // 同一 pdo_id
  node.add_pdo_tx({0x200, 0, 0, 0, 0, 100, 0});
  node.add_pdo_tx({0x200, 1, 0, 0, 0, 200, 0});  // 同一 pdo_id

  EXPECT_EQ(node.pdo_rx_count(), 3u);
  EXPECT_EQ(node.pdo_tx_count(), 2u);

  node.clear_pdo(0x100);
  EXPECT_EQ(node.pdo_rx_count(), 1u);
  EXPECT_EQ(node.pdo_rx_at(0).pdo_id, 0x101u);  // 残ったもの

  node.clear_pdo(0x200);
  EXPECT_EQ(node.pdo_tx_count(), 0u);
}

// ════════════════════════════════════════════════════════════════
// reset_pdos: 全消去
// ════════════════════════════════════════════════════════════════

TEST(NodeBaseTest, ResetPdos)
{
  TestNode node(1);

  node.add_pdo_rx({0x100, 0, 0, 0, 4});
  node.add_pdo_tx({0x200, 0, 0, 0, 0, 100, 0});
  EXPECT_EQ(node.pdo_rx_count(), 1u);
  EXPECT_EQ(node.pdo_tx_count(), 1u);

  node.reset_pdos();
  EXPECT_EQ(node.pdo_rx_count(), 0u);
  EXPECT_EQ(node.pdo_tx_count(), 0u);
}

TEST(NodeBaseTest, RequestPdoTxSetAndClear)
{
  TestNode node(1);

  node.add_pdo_tx({0x200, 0, 0, 0, 0, 100, 0});
  node.add_pdo_tx({0x200, 1, 0, 0, 0, 100, 0});

  EXPECT_FALSE(node.is_pdo_tx_requested(0x200));
  EXPECT_EQ(node.request_pdo_tx(0x200), Status::OK);
  EXPECT_TRUE(node.is_pdo_tx_requested(0x200));

  node.clear_pdo_tx_request(0x200);
  EXPECT_FALSE(node.is_pdo_tx_requested(0x200));
}

TEST(NodeBaseTest, RequestPdoTxNotFound)
{
  TestNode node(1);
  node.add_pdo_tx({0x201, 0, 0, 0, 0, 100, 0});
  EXPECT_EQ(node.request_pdo_tx(0x200), Status::NOT_FOUND);
}

TEST(NodeBaseTest, RequestFlagRemovedByClearAndReset)
{
  TestNode node(1);

  node.add_pdo_tx({0x200, 0, 0, 0, 0, 100, 0});
  node.add_pdo_tx({0x201, 0, 0, 0, 0, 100, 0});
  EXPECT_EQ(node.request_pdo_tx(0x200), Status::OK);
  EXPECT_EQ(node.request_pdo_tx(0x201), Status::OK);

  node.clear_pdo(0x200);
  EXPECT_FALSE(node.is_pdo_tx_requested(0x200));
  EXPECT_TRUE(node.is_pdo_tx_requested(0x201));

  node.reset_pdos();
  EXPECT_FALSE(node.is_pdo_tx_requested(0x201));
}
