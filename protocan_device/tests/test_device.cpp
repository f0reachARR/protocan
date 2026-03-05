#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "protocan/can_frame.hpp"
#include "protocan/types.hpp"
#include "protocan_device/device.hpp"
#include "protocan_device/node_base.hpp"

using namespace protocan;
using namespace protocan::device;

// ════════════════════════════════════════════════════════════════
// テスト用モック
// ════════════════════════════════════════════════════════════════

class MockCan : public ICanInterface
{
public:
  std::vector<CanFrame> sent;
  std::vector<CanFrame> rx_queue;

  Status send(const CanFrame & f) override
  {
    sent.push_back(f);
    return Status::OK;
  }

  bool try_receive(CanFrame & out) override
  {
    if (rx_queue.empty()) return false;
    out = rx_queue.front();
    rx_queue.erase(rx_queue.begin());
    return true;
  }

  void push_rx(const CanFrame & f) { rx_queue.push_back(f); }
  void clear() { sent.clear(); rx_queue.clear(); }
};

// シンプルなディスクリプタBlobテスト用
static const uint8_t kTestBlob[] = {0x01, 0x02, 0x03, 0x04};

class MockNode : public NodeBase
{
public:
  MockNode(uint8_t id)
  : NodeBase(id, "mock_node", kTestBlob, sizeof(kTestBlob), 0x12345678u)
  {
  }

  uint16_t last_pdo_rx_id = 0;
  uint8_t last_pdo_rx_data[64] = {};
  uint8_t last_pdo_rx_len = 0;

  void on_pdo_rx(uint16_t pdo_id, const uint8_t * data, uint8_t len) override
  {
    last_pdo_rx_id = pdo_id;
    last_pdo_rx_len = len;
    if (len > 0) std::memcpy(last_pdo_rx_data, data, len);
  }

  uint8_t fill_pdo_tx(uint16_t, uint8_t * buf, uint8_t) override
  {
    buf[0] = 0xAB;
    buf[1] = 0xCD;
    return 2;
  }

  Status on_param_read(uint8_t idx, uint8_t * out, uint8_t & out_size) override
  {
    if (idx == 0) {
      out[0] = 42;
      out_size = 1;
      return Status::OK;
    }
    return Status::NOT_FOUND;
  }

  Status on_param_write(uint8_t idx, const uint8_t * data, uint8_t size) override
  {
    if (idx == 0 && size >= 1) {
      written_value = data[0];
      return Status::OK;
    }
    return Status::NOT_FOUND;
  }

  Status on_service_req(
    uint8_t svc_idx, const uint8_t *, uint8_t, uint8_t * res, uint8_t & res_size) override
  {
    if (svc_idx == 0) {
      res[0] = 0xFF;
      res_size = 1;
      return Status::OK;
    }
    return Status::NOT_FOUND;
  }

  uint8_t written_value = 0;
};

// 時刻管理
static uint32_t g_time_ms = 0;
static uint32_t get_time() { return g_time_ms; }
static uint32_t rand_ms_fixed(uint32_t min_ms, uint32_t) { return min_ms; }

// ════════════════════════════════════════════════════════════════
// テストフィクスチャ
// ════════════════════════════════════════════════════════════════

class DeviceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    g_time_ms = 0;
    can.clear();
  }

  MockCan can;
  MockNode node{2};

  Device make_device(uint8_t dev_id = 3)
  {
    return Device(dev_id, &can, get_time, rand_ms_fixed);
  }

  /// ExtendedId から CanFrame を作成
  static CanFrame make_nmt_frame(uint8_t src_dev, uint8_t context, const uint8_t * data = nullptr, uint8_t len = 0)
  {
    ExtendedId eid{};
    eid.function_code = FunctionCode::NMT;
    eid.src_dev = src_dev;
    eid.src_node = 0;
    eid.dst_dev = kBroadcastDeviceId;
    eid.dst_node = kBroadcastNodeId;
    eid.context = context;
    return make_extended_frame(eid, data, len);
  }

  static CanFrame make_nmt_cmd(uint8_t dst_dev, NmtCommand cmd)
  {
    ExtendedId eid{};
    eid.function_code = FunctionCode::NMT;
    eid.src_dev = kMasterDeviceId;
    eid.src_node = 0;
    eid.dst_dev = dst_dev;
    eid.dst_node = kBroadcastNodeId;
    eid.context = static_cast<uint8_t>(cmd);
    return make_extended_frame(eid, nullptr, 0);
  }

  static CanFrame make_bulk_fc(uint8_t dst_dev, uint8_t channel, BulkFcFlag flag)
  {
    ExtendedId eid{};
    eid.function_code = FunctionCode::BULK;
    eid.src_dev = kMasterDeviceId;
    eid.src_node = 0;
    eid.dst_dev = dst_dev;
    eid.dst_node = 0;
    eid.context = channel;

    uint8_t payload[5] = {};
    payload[0] = static_cast<uint8_t>(BulkFrameType::FLOW_CONTROL);
    payload[1] = static_cast<uint8_t>(flag);
    payload[2] = 0;  // block_size = unlimited
    payload[3] = 0;
    payload[4] = 0;
    return make_extended_frame(eid, payload, 5);
  }
};

// ════════════════════════════════════════════════════════════════
// 1. start() / poll() 状態遷移
// ════════════════════════════════════════════════════════════════

TEST_F(DeviceTest, StartEntersPreopListenOnly)
{
  Device dev = make_device();
  dev.add_node(node);
  dev.start();

  // Listen-Only 期間: ハートビートを送らない
  g_time_ms = 50;
  dev.poll();
  EXPECT_EQ(can.sent.size(), 0u);
  EXPECT_EQ(dev.state(), DeviceState::PREOP);
}

TEST_F(DeviceTest, PollSendsHeartbeatAfterListenOnly)
{
  Device dev = make_device(3);
  dev.add_node(node);
  dev.start();

  // Listen-Only 最小期間 (100ms) 経過後
  g_time_ms = 101;
  dev.poll();

  // ハートビートが送信されること
  ASSERT_GE(can.sent.size(), 1u);
  const CanFrame & hb = can.sent[0];
  EXPECT_TRUE(hb.is_extended);

  ExtendedId eid = decode_extended_id(hb.id);
  EXPECT_EQ(eid.function_code, FunctionCode::NMT);
  EXPECT_EQ(eid.src_dev, 3u);
  EXPECT_EQ(eid.context, 0u);  // heartbeat

  // PREOP ハートビートはノード情報を含む
  EXPECT_EQ(hb.data[0], static_cast<uint8_t>(DeviceState::PREOP));
  EXPECT_EQ(hb.data[1], 1u);  // num_nodes = 1
  // schema_hash at offset 6
  uint32_t schema_hash = static_cast<uint32_t>(hb.data[6]) |
                         (static_cast<uint32_t>(hb.data[7]) << 8) |
                         (static_cast<uint32_t>(hb.data[8]) << 16) |
                         (static_cast<uint32_t>(hb.data[9]) << 24);
  EXPECT_EQ(schema_hash, 0x12345678u);
  EXPECT_EQ(hb.data[10], 2u);  // local_node_id
}

TEST_F(DeviceTest, NmtStartTransitionsToOperational)
{
  Device dev = make_device(3);
  dev.add_node(node);
  dev.start();

  // Listen-Only 解除
  g_time_ms = 101;
  dev.poll();
  can.clear();

  // NMT START 受信
  can.push_rx(make_nmt_cmd(kBroadcastDeviceId, NmtCommand::START));
  g_time_ms = 200;
  dev.poll();

  EXPECT_EQ(dev.state(), DeviceState::OPERATIONAL);

  // OPERATIONAL ハートビートはノード情報なし
  ASSERT_GE(can.sent.size(), 1u);
  const CanFrame & hb = can.sent[0];
  EXPECT_EQ(hb.data[0], static_cast<uint8_t>(DeviceState::OPERATIONAL));
  EXPECT_EQ(hb.data[1], 0u);  // num_nodes = 0
}

TEST_F(DeviceTest, NmtEnterPreopFromOperational)
{
  Device dev = make_device(3);
  dev.add_node(node);
  dev.start();
  g_time_ms = 101;
  dev.poll();
  can.clear();

  // OPERATIONAL へ遷移
  can.push_rx(make_nmt_cmd(3, NmtCommand::START));
  dev.poll();
  ASSERT_EQ(dev.state(), DeviceState::OPERATIONAL);
  can.clear();

  // ENTER_PREOP
  can.push_rx(make_nmt_cmd(kBroadcastDeviceId, NmtCommand::ENTER_PREOP));
  g_time_ms = 300;
  dev.poll();
  EXPECT_EQ(dev.state(), DeviceState::PREOP);
}

// ════════════════════════════════════════════════════════════════
// 2. ID衝突検出
// ════════════════════════════════════════════════════════════════

TEST_F(DeviceTest, IdCollisionDuringListenOnly)
{
  Device dev = make_device(5);
  dev.start();

  // Listen-Only 中に device_id=5 のハートビートを受信
  uint8_t hb_payload[6] = {static_cast<uint8_t>(DeviceState::PREOP), 0, 0, 0, 0, 0};
  can.push_rx(make_nmt_frame(5, 0, hb_payload, 6));

  g_time_ms = 50;
  dev.poll();
  EXPECT_EQ(dev.state(), DeviceState::ERROR);
}

TEST_F(DeviceTest, IdCollisionDuringInitialHb)
{
  Device dev = make_device(5);
  dev.start();

  // Listen-Only 解除
  g_time_ms = 101;
  dev.poll();  // first heartbeat (initial_hb_count = 1)
  can.clear();

  // initial_hb_count < 3 中に衝突
  uint8_t hb_payload[6] = {static_cast<uint8_t>(DeviceState::PREOP), 0, 0, 0, 0, 0};
  can.push_rx(make_nmt_frame(5, 0, hb_payload, 6));

  g_time_ms = 150;
  dev.poll();
  EXPECT_EQ(dev.state(), DeviceState::ERROR);
}

// ════════════════════════════════════════════════════════════════
// 3. DISC_GET_DESCRIPTOR → BULK 応答
// ════════════════════════════════════════════════════════════════

TEST_F(DeviceTest, DiscGetDescriptorStartsBulk)
{
  Device dev = make_device(3);
  dev.add_node(node);
  dev.start();
  g_time_ms = 101;
  dev.poll();
  can.clear();

  // DISC_GET_DESCRIPTOR 受信 (dst_node = node.local_id = 2)
  ExtendedId disc_eid{};
  disc_eid.function_code = FunctionCode::DISC;
  disc_eid.src_dev = kMasterDeviceId;
  disc_eid.src_node = 0;
  disc_eid.dst_dev = 3;
  disc_eid.dst_node = 2;
  disc_eid.context = 0;
  can.push_rx(make_extended_frame(disc_eid, nullptr, 0));

  dev.poll();

  // First Frame が送信されていること
  bool found_ff = false;
  for (const auto & f : can.sent) {
    if (!f.is_extended) continue;
    ExtendedId eid = decode_extended_id(f.id);
    if (eid.function_code == FunctionCode::BULK && eid.src_dev == 3u) {
      EXPECT_EQ(f.data[0], static_cast<uint8_t>(BulkFrameType::FIRST_FRAME));
      EXPECT_EQ(f.data[1], static_cast<uint8_t>(BulkPayloadType::DESCRIPTOR));
      // total_length = 4 (sizeof kTestBlob)
      uint32_t total = static_cast<uint32_t>(f.data[2]) | (static_cast<uint32_t>(f.data[3]) << 8) |
                       (static_cast<uint32_t>(f.data[4]) << 16) | (static_cast<uint32_t>(f.data[5]) << 24);
      EXPECT_EQ(total, 4u);
      // kTestBlob (4 bytes) fits in First Frame
      EXPECT_EQ(f.data[6], kTestBlob[0]);
      EXPECT_EQ(f.data[7], kTestBlob[1]);
      EXPECT_EQ(f.data[8], kTestBlob[2]);
      EXPECT_EQ(f.data[9], kTestBlob[3]);
      found_ff = true;
    }
  }
  EXPECT_TRUE(found_ff);
}

TEST_F(DeviceTest, BulkLargerThan58BytesSendsConsecutive)
{
  // 大きな blob を持つノード
  static uint8_t large_blob[100];
  for (int i = 0; i < 100; ++i) large_blob[i] = static_cast<uint8_t>(i);

  class BigNode : public NodeBase
  {
  public:
    BigNode()
    : NodeBase(10, "big", large_blob, 100, 0xAABBCCDDu)
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
  } big_node;

  Device dev = make_device(3);
  dev.add_node(big_node);
  dev.start();
  g_time_ms = 101;
  dev.poll();
  can.clear();

  // DISC_GET_DESCRIPTOR
  ExtendedId disc_eid{};
  disc_eid.function_code = FunctionCode::DISC;
  disc_eid.src_dev = kMasterDeviceId;
  disc_eid.src_node = 0;
  disc_eid.dst_dev = 3;
  disc_eid.dst_node = 10;
  disc_eid.context = 0;
  can.push_rx(make_extended_frame(disc_eid, nullptr, 0));

  dev.poll();
  can.clear();

  // FC CONTINUE を送信してConsecutive Frame を受け取る
  can.push_rx(make_bulk_fc(3, 0, BulkFcFlag::CONTINUE));
  dev.poll();

  // Consecutive Frame が送信されていること
  bool found_cf = false;
  for (const auto & f : can.sent) {
    if (!f.is_extended) continue;
    ExtendedId eid = decode_extended_id(f.id);
    if (eid.function_code == FunctionCode::BULK) {
      if (f.data[0] == static_cast<uint8_t>(BulkFrameType::CONSECUTIVE_FRAME)) {
        found_cf = true;
        break;
      }
    }
  }
  EXPECT_TRUE(found_cf);
}

// ════════════════════════════════════════════════════════════════
// 4. PDO_CFG トランザクション
// ════════════════════════════════════════════════════════════════

TEST_F(DeviceTest, PdoCfgApplyCreatesRxEntry)
{
  Device dev = make_device(3);
  dev.add_node(node);
  dev.start();
  g_time_ms = 101;
  dev.poll();
  can.clear();

  auto send_pdo_cfg_frame = [&](uint8_t ctx, const uint8_t * payload, uint8_t len) {
    ExtendedId eid{};
    eid.function_code = FunctionCode::PDO_CFG;
    eid.src_dev = kMasterDeviceId;
    eid.src_node = 0;
    eid.dst_dev = 3;
    eid.dst_node = kBroadcastNodeId;
    eid.context = ctx;
    can.push_rx(make_extended_frame(eid, payload, len));
  };

  // BEGIN
  uint8_t begin[7] = {0x50, 0x01,  // pdo_id = 0x0150
                      1,           // direction = RX
                      1,           // num_entries
                      100, 0,      // period_ms = 100
                      8};          // total_size
  send_pdo_cfg_frame(0, begin, 7);

  // ENTRY
  uint8_t entry[5] = {2,  // local_node_id = 2 (node.local_id)
                      0,  // topic_index
                      0,  // field_index
                      0,  // offset
                      8}; // size
  send_pdo_cfg_frame(1, entry, 5);

  // COMMIT APPLY
  uint8_t commit[3] = {0x50, 0x01, 0};  // pdo_id=0x0150, action=APPLY
  send_pdo_cfg_frame(0x1F, commit, 3);

  dev.poll();

  // ACK が送信されること
  bool ack_sent = false;
  for (const auto & f : can.sent) {
    if (!f.is_extended) continue;
    ExtendedId eid = decode_extended_id(f.id);
    if (eid.function_code == FunctionCode::PDO_CFG && eid.src_dev == 3u) {
      uint16_t pdo_id = static_cast<uint16_t>(f.data[0]) | (static_cast<uint16_t>(f.data[1]) << 8);
      EXPECT_EQ(pdo_id, 0x0150u);
      EXPECT_EQ(f.data[2], static_cast<uint8_t>(PdoCfgStatus::OK));
      ack_sent = true;
    }
  }
  EXPECT_TRUE(ack_sent);

  // ノードに RX エントリが追加されていること
  EXPECT_EQ(node.pdo_rx_count(), 1u);
  EXPECT_EQ(node.pdo_rx_at(0).pdo_id, 0x0150u);
}

TEST_F(DeviceTest, PdoCfgDeleteClearsEntries)
{
  Device dev = make_device(3);
  dev.add_node(node);
  dev.start();
  g_time_ms = 101;
  dev.poll();

  // 直接エントリを追加
  node.add_pdo_rx({0x300, 0, 0, 0, 4});
  EXPECT_EQ(node.pdo_rx_count(), 1u);
  can.clear();

  auto send_commit_delete = [&](uint16_t pdo_id) {
    ExtendedId eid{};
    eid.function_code = FunctionCode::PDO_CFG;
    eid.src_dev = kMasterDeviceId;
    eid.src_node = 0;
    eid.dst_dev = 3;
    eid.dst_node = kBroadcastNodeId;
    eid.context = static_cast<uint8_t>(PdoCfgSequence::COMMIT);

    uint8_t payload[3] = {};
    payload[0] = static_cast<uint8_t>(pdo_id);
    payload[1] = static_cast<uint8_t>(pdo_id >> 8);
    payload[2] = static_cast<uint8_t>(PdoCfgAction::DELETE);
    can.push_rx(make_extended_frame(eid, payload, 3));
  };

  send_commit_delete(0x300);
  dev.poll();

  EXPECT_EQ(node.pdo_rx_count(), 0u);
}

// ════════════════════════════════════════════════════════════════
// 5. PARAM READ/WRITE
// ════════════════════════════════════════════════════════════════

TEST_F(DeviceTest, ParamRead)
{
  Device dev = make_device(3);
  dev.add_node(node);
  dev.start();
  g_time_ms = 101;
  dev.poll();
  can.clear();

  ExtendedId eid{};
  eid.function_code = FunctionCode::PARAM;
  eid.src_dev = kMasterDeviceId;
  eid.src_node = 0;
  eid.dst_dev = 3;
  eid.dst_node = 2;  // node.local_id
  eid.context = static_cast<uint8_t>(ParamCommand::READ);

  uint8_t payload[1] = {0};  // param_index = 0
  can.push_rx(make_extended_frame(eid, payload, 1));

  dev.poll();

  // READ_RES が送信されること
  bool res_found = false;
  for (const auto & f : can.sent) {
    if (!f.is_extended) continue;
    ExtendedId res_eid = decode_extended_id(f.id);
    if (res_eid.function_code == FunctionCode::PARAM &&
        res_eid.context == static_cast<uint8_t>(ParamCommand::READ_RES)) {
      EXPECT_EQ(f.data[0], 0u);  // param_index
      EXPECT_EQ(f.data[1], static_cast<uint8_t>(ParamStatus::OK));
      EXPECT_EQ(f.data[2], 42u);  // value
      res_found = true;
    }
  }
  EXPECT_TRUE(res_found);
}

TEST_F(DeviceTest, ParamWrite)
{
  Device dev = make_device(3);
  dev.add_node(node);
  dev.start();
  g_time_ms = 101;
  dev.poll();
  can.clear();

  ExtendedId eid{};
  eid.function_code = FunctionCode::PARAM;
  eid.src_dev = kMasterDeviceId;
  eid.src_node = 0;
  eid.dst_dev = 3;
  eid.dst_node = 2;
  eid.context = static_cast<uint8_t>(ParamCommand::WRITE);

  uint8_t payload[2] = {0, 99};  // param_index=0, value=99
  can.push_rx(make_extended_frame(eid, payload, 2));

  dev.poll();

  EXPECT_EQ(node.written_value, 99u);

  bool res_found = false;
  for (const auto & f : can.sent) {
    if (!f.is_extended) continue;
    ExtendedId res_eid = decode_extended_id(f.id);
    if (res_eid.function_code == FunctionCode::PARAM &&
        res_eid.context == static_cast<uint8_t>(ParamCommand::WRITE_RES)) {
      EXPECT_EQ(f.data[1], static_cast<uint8_t>(ParamStatus::OK));
      res_found = true;
    }
  }
  EXPECT_TRUE(res_found);
}

// ════════════════════════════════════════════════════════════════
// 6. SERVICE
// ════════════════════════════════════════════════════════════════

TEST_F(DeviceTest, ServiceRequest)
{
  Device dev = make_device(3);
  dev.add_node(node);
  dev.start();
  g_time_ms = 101;
  dev.poll();

  // OPERATIONAL へ
  can.push_rx(make_nmt_cmd(3, NmtCommand::START));
  dev.poll();
  can.clear();

  ExtendedId eid{};
  eid.function_code = FunctionCode::SERVICE;
  eid.src_dev = kMasterDeviceId;
  eid.src_node = 0;
  eid.dst_dev = 3;
  eid.dst_node = 2;
  eid.context = 7;  // seq_id

  uint8_t payload[1] = {0};  // svc_idx = 0
  can.push_rx(make_extended_frame(eid, payload, 1));

  dev.poll();

  bool res_found = false;
  for (const auto & f : can.sent) {
    if (!f.is_extended) continue;
    ExtendedId res_eid = decode_extended_id(f.id);
    if (res_eid.function_code == FunctionCode::SERVICE && res_eid.src_dev == 3u) {
      EXPECT_EQ(res_eid.context, 7u);  // 同一 seq_id
      EXPECT_EQ(f.data[0], 0u);  // svc_idx
      EXPECT_EQ(f.data[1], static_cast<uint8_t>(ServiceStatus::OK));
      EXPECT_EQ(f.data[2], 0xFFu);  // response data
      res_found = true;
    }
  }
  EXPECT_TRUE(res_found);
}

// ════════════════════════════════════════════════════════════════
// 7. EMCY 送信
// ════════════════════════════════════════════════════════════════

TEST_F(DeviceTest, SendEmcy)
{
  Device dev = make_device(3);
  dev.add_node(node);
  dev.start();
  g_time_ms = 101;
  dev.poll();
  can.clear();

  uint8_t err_data[] = {0x01, 0x02};
  Status s = dev.send_emcy(2, 0x0001, err_data, 2);
  EXPECT_EQ(s, Status::OK);

  ASSERT_EQ(can.sent.size(), 1u);
  const CanFrame & f = can.sent[0];
  ExtendedId eid = decode_extended_id(f.id);
  EXPECT_EQ(eid.function_code, FunctionCode::EMCY);
  EXPECT_EQ(eid.src_dev, 3u);
  EXPECT_EQ(eid.src_node, 2u);
  // error_register = 0x0001 (LE)
  EXPECT_EQ(f.data[0], 0x01u);
  EXPECT_EQ(f.data[1], 0x00u);
  EXPECT_EQ(f.data[2], 0x01u);
  EXPECT_EQ(f.data[3], 0x02u);
}

// ════════════════════════════════════════════════════════════════
// 8. PDO RX 受信 (OPERATIONAL)
// ════════════════════════════════════════════════════════════════

TEST_F(DeviceTest, PdoRxDeliveredToNode)
{
  Device dev = make_device(3);
  dev.add_node(node);
  dev.start();
  g_time_ms = 101;
  dev.poll();

  // OPERATIONAL へ
  can.push_rx(make_nmt_cmd(3, NmtCommand::START));
  dev.poll();

  // PDO RX エントリを追加
  node.add_pdo_rx({0x100, 0, 0, 0, 4});

  // PDO フレーム受信
  uint8_t pdo_data[] = {0x11, 0x22, 0x33, 0x44};
  CanFrame pdo_frame = make_standard_frame(0x100, pdo_data, 4);
  can.push_rx(pdo_frame);

  g_time_ms = 200;
  dev.poll();

  EXPECT_EQ(node.last_pdo_rx_id, 0x100u);
  EXPECT_EQ(node.last_pdo_rx_len, 4u);
  EXPECT_EQ(node.last_pdo_rx_data[0], 0x11u);
}

// ════════════════════════════════════════════════════════════════
// 9. add_node 上限
// ════════════════════════════════════════════════════════════════

TEST_F(DeviceTest, AddNodeLimit)
{
  Device dev = make_device(3);

  // kMaxNodes 個のモックノードを作る
  static uint8_t blob[] = {0};
  class MinNode : public NodeBase
  {
  public:
    MinNode(uint8_t id) : NodeBase(id, "", blob, 1, 0) {}
    void on_pdo_rx(uint16_t, const uint8_t *, uint8_t) override {}
    uint8_t fill_pdo_tx(uint16_t, uint8_t *, uint8_t) override { return 0; }
    Status on_param_read(uint8_t, uint8_t *, uint8_t &) override { return Status::NOT_FOUND; }
    Status on_param_write(uint8_t, const uint8_t *, uint8_t) override { return Status::NOT_FOUND; }
    Status on_service_req(uint8_t, const uint8_t *, uint8_t, uint8_t *, uint8_t &) override
    {
      return Status::NOT_FOUND;
    }
  };

  static MinNode nodes[kMaxNodes + 1] = {
    MinNode(1), MinNode(2), MinNode(3), MinNode(4), MinNode(5), MinNode(6),
    MinNode(7), MinNode(8), MinNode(9), MinNode(10), MinNode(11), MinNode(12),
    MinNode(13), MinNode(14), MinNode(15), MinNode(16), MinNode(17)
  };

  for (uint8_t i = 0; i < kMaxNodes; ++i) {
    EXPECT_EQ(dev.add_node(nodes[i]), Status::OK);
  }
  EXPECT_EQ(dev.add_node(nodes[kMaxNodes]), Status::NO_RESOURCE);
}
