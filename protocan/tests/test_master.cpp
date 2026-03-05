#include <gtest/gtest.h>

#include <cstring>

#include "protocan/descriptor.pb.h"
#include "protocan/master.hpp"

using namespace protocan;

// ── LE ペイロード書き込みヘルパー ──
static inline void write_le32(uint8_t * p, uint32_t v)
{
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}

// ── Mock CAN Interface ──
class MockCanInterface : public ICanInterface
{
public:
  std::vector<CanFrame> sent_frames;
  std::vector<CanFrame> rx_queue;

  Status send(const CanFrame & frame) override
  {
    sent_frames.push_back(frame);
    return Status::OK;
  }

  std::optional<CanFrame> receive() override
  {
    if (rx_queue.empty()) return std::nullopt;
    auto frame = rx_queue.front();
    rx_queue.erase(rx_queue.begin());
    return frame;
  }

  Status open() override { return Status::OK; }
  Status close() override { return Status::OK; }
  bool is_open() const override { return true; }

  void push_rx(const CanFrame & frame) { rx_queue.push_back(frame); }

  void clear()
  {
    sent_frames.clear();
    rx_queue.clear();
  }
};

class MasterTest : public ::testing::Test
{
protected:
  MockCanInterface can;
  MasterCallbacks callbacks;
};

// ════════════════════════════════════════════════════════════════
// シナリオ 1: デバイス発見 (Discovery)
// ════════════════════════════════════════════════════════════════
TEST_F(MasterTest, DeviceDiscoveryScenario)
{
  bool device_discovered = false;
  callbacks.on_device_discovered = [&](uint8_t dev_id, const DeviceInfo & info) {
    EXPECT_EQ(dev_id, 5);
    EXPECT_EQ(info.nodes.size(), 1u);
    EXPECT_EQ(info.nodes[0].local_node_id, 2);
    EXPECT_EQ(info.nodes[0].schema_hash, 0xDEADBEEF);
    device_discovered = true;
  };

  Master master(can, callbacks);

  // デバイス(Device ID=5) が HEARTBEAT を送信したと仮定
  // Node: 1個 (local_node_id=2, schema_hash=0xDEADBEEF)
  ExtendedId eid;
  eid.function_code = FunctionCode::NMT;
  eid.src_dev = 5;
  eid.src_node = 0;
  eid.dst_dev = kBroadcastDeviceId;
  eid.dst_node = kBroadcastNodeId;
  eid.context = 0;

  uint8_t payload[14] = {};
  payload[0] = static_cast<uint8_t>(DeviceState::PREOP);
  payload[1] = 1;                       // 1 node
  write_le32(payload + 2, 1000);        // uptime_ms
  write_le32(payload + 6, 0xDEADBEEF);  // schema_hash
  payload[10] = 2;                      // local_node_id

  can.push_rx(make_extended_frame(eid, payload, 14));

  // Master で受信処理
  master.poll();

  // コールバックが発火したはず
  EXPECT_TRUE(device_discovered);

  // Master は未知の schema_hash (0xDEADBEEF) に対して DISC (Get Descriptor) を返信したはず
  ASSERT_EQ(can.sent_frames.size(), 1u);
  auto reply = can.sent_frames[0];
  EXPECT_TRUE(reply.is_extended);
  ExtendedId rep_eid = decode_extended_id(reply.id);
  EXPECT_EQ(rep_eid.function_code, FunctionCode::DISC);
  EXPECT_EQ(rep_eid.src_dev, kMasterDeviceId);
  EXPECT_EQ(rep_eid.dst_dev, 5);
  EXPECT_EQ(rep_eid.dst_node, 2);  // 対象の local_node_id にリクエスト
}

// ════════════════════════════════════════════════════════════════
// シナリオ 2: Descriptor Payload 受信 (BULK転送シミュレーション)
// ════════════════════════════════════════════════════════════════
TEST_F(MasterTest, BulkDescriptorReceiveScenario)
{
  bool descriptor_received = false;
  callbacks.on_descriptor_received =
    [&](uint8_t dev_id, uint8_t node_id, const ParsedDescriptor & desc) {
      EXPECT_EQ(dev_id, 3);
      EXPECT_EQ(node_id, 1);
      EXPECT_EQ(desc.node_type_name, "DummyNode");
      descriptor_received = true;
    };
  Master master(can, callbacks);

  // ダミーの NodeDescriptor を作成して Protobuf でシリアライズ
  protocan::NodeDescriptor pb_desc;
  pb_desc.set_schema_hash(0x12345678);
  pb_desc.set_node_type_name("DummyNode");

  std::string blob;
  pb_desc.SerializeToString(&blob);

  // デバイス(ID=3, Node=1)が BULK転送で送信してきたと仮定
  ExtendedId eid;
  eid.function_code = FunctionCode::BULK;
  eid.src_dev = 3;
  eid.src_node = 1;
  eid.dst_dev = kMasterDeviceId;
  eid.dst_node = 0;
  eid.context = 0;  // Channel 0

  // First Frame に収まるサイズと仮定
  uint8_t ff_payload[64] = {};
  ff_payload[0] = static_cast<uint8_t>(BulkFrameType::FIRST_FRAME);
  ff_payload[1] = static_cast<uint8_t>(BulkPayloadType::DESCRIPTOR);
  write_le32(ff_payload + 2, blob.size());
  std::memcpy(ff_payload + 6, blob.data(), blob.size());

  can.push_rx(make_extended_frame(eid, ff_payload, static_cast<uint8_t>(6 + blob.size())));

  master.poll();  // 受信処理

  // (テスト設定として、FirstFrame に収まった場合はすぐに完了する)
  EXPECT_TRUE(descriptor_received);

  // Schema Hash がキャッシュされたか確認
  EXPECT_TRUE(master.device_tracker().is_schema_known(0x12345678));
}

// ════════════════════════════════════════════════════════════════
// シナリオ 3: Parameter の Read/Write トランザクション
// ════════════════════════════════════════════════════════════════
TEST_F(MasterTest, ParameterScenario)
{
  bool param_res = false;
  callbacks.on_param_response = [&](
                                  uint8_t dev_id, uint8_t node_id, ParamCommand cmd,
                                  uint8_t param_idx, ParamStatus st, const uint8_t * val,
                                  size_t len) {
    EXPECT_EQ(dev_id, 4);
    EXPECT_EQ(node_id, 0);
    EXPECT_EQ(cmd, ParamCommand::READ_RES);
    EXPECT_EQ(param_idx, 5);
    EXPECT_EQ(st, ParamStatus::OK);
    ASSERT_EQ(len, 4u);
    EXPECT_EQ(val[0], 0xAA);
    param_res = true;
  };

  Master master(can, callbacks);

  // マスターから READ 要求を送信
  Status s = master.send_param_read(4, 0, 5);
  EXPECT_EQ(s, Status::OK);

  ASSERT_EQ(can.sent_frames.size(), 1u);
  ExtendedId sent_eid = decode_extended_id(can.sent_frames[0].id);
  EXPECT_EQ(sent_eid.function_code, FunctionCode::PARAM);
  EXPECT_EQ(sent_eid.context, static_cast<uint8_t>(ParamCommand::READ));

  can.clear();

  // デバイスからの応答をシミュレート
  ExtendedId rep_eid;
  rep_eid.function_code = FunctionCode::PARAM;
  rep_eid.src_dev = 4;
  rep_eid.src_node = 0;
  rep_eid.dst_dev = kMasterDeviceId;
  rep_eid.dst_node = 0;
  rep_eid.context = static_cast<uint8_t>(ParamCommand::READ_RES);

  uint8_t rep_data[] = {5 /* idx */, 0 /* ok */, 0xAA, 0xBB, 0xCC, 0xDD};  // value
  can.push_rx(make_extended_frame(rep_eid, rep_data, 6));

  master.poll();
  EXPECT_TRUE(param_res);
}

// ════════════════════════════════════════════════════════════════
// シナリオ 4: PDO_CFG マッピングの発行
// ════════════════════════════════════════════════════════════════
TEST_F(MasterTest, PdoCfgScenario)
{
  Master master(can, {});

  PdoMapping mapping;
  mapping.pdo_id = 0x123;
  mapping.direction = PdoCfgDirection::TX;
  mapping.period_ms = 100;
  mapping.total_size = 8;
  mapping.entries.push_back({0, 1, 0, 0, 4});  // node=1, topic=0, field=0, offset=0, size=4
  mapping.entries.push_back({0, 1, 1, 4, 4});  // node=1, topic=0, field=1, offset=4, size=4

  Status s = master.send_pdo_cfg(2, mapping);
  EXPECT_EQ(s, Status::OK);

  // BEGIN, ENTRY, ENTRY, COMMIT の4フレームが発行されるはず
  ASSERT_EQ(can.sent_frames.size(), 4u);

  // 1. BEGIN
  ExtendedId eid_begin = decode_extended_id(can.sent_frames[0].id);
  EXPECT_EQ(eid_begin.function_code, FunctionCode::PDO_CFG);
  EXPECT_EQ(eid_begin.context, static_cast<uint8_t>(PdoCfgSequence::BEGIN));

  // 2. ENTRY
  ExtendedId eid_entry1 = decode_extended_id(can.sent_frames[1].id);
  EXPECT_EQ(eid_entry1.context, 1);

  // 3. ENTRY
  ExtendedId eid_entry2 = decode_extended_id(can.sent_frames[2].id);
  EXPECT_EQ(eid_entry2.context, 2);

  // 4. COMMIT
  ExtendedId eid_commit = decode_extended_id(can.sent_frames[3].id);
  EXPECT_EQ(eid_commit.context, static_cast<uint8_t>(PdoCfgSequence::COMMIT));

  // PDO Manager の内部状態にもマッピングが登録されているか確認
  auto registered = master.pdo_manager().get_mapping(0x123);
  EXPECT_TRUE(registered.has_value());
  EXPECT_EQ(registered->entries.size(), 2u);
}

// ════════════════════════════════════════════════════════════════
// シナリオ 5: Service リクエスト・レスポンス (単一フレーム)
// ════════════════════════════════════════════════════════════════
TEST_F(MasterTest, ServiceSingleFrameScenario)
{
  bool service_res = false;
  callbacks.on_service_response = [&](
                                    uint8_t dev_id, uint8_t node_id, uint8_t svc_idx,
                                    ServiceStatus st, const uint8_t * val, size_t len) {
    EXPECT_EQ(dev_id, 6);
    EXPECT_EQ(node_id, 2);
    EXPECT_EQ(svc_idx, 3);
    EXPECT_EQ(st, ServiceStatus::OK);
    EXPECT_EQ(len, 2u);
    EXPECT_EQ(val[0], 0x11);
    EXPECT_EQ(val[1], 0x22);
    service_res = true;
  };
  Master master(can, callbacks);

  uint8_t req_data[] = {0xFF};
  auto seq_id_opt = master.send_service_request(6, 2, 3, req_data, 1);
  ASSERT_TRUE(seq_id_opt.has_value());
  uint8_t seq_id = *seq_id_opt;

  // フレーム送信確認
  ASSERT_EQ(can.sent_frames.size(), 1u);
  ExtendedId sent_eid = decode_extended_id(can.sent_frames[0].id);
  EXPECT_EQ(sent_eid.function_code, FunctionCode::SERVICE);
  EXPECT_EQ(sent_eid.context, seq_id);

  // デバイスからのレスポンスをシミュレート
  ExtendedId rep_eid;
  rep_eid.function_code = FunctionCode::SERVICE;
  rep_eid.src_dev = 6;
  rep_eid.src_node = 2;
  rep_eid.dst_dev = kMasterDeviceId;
  rep_eid.dst_node = 0;
  rep_eid.context = seq_id;

  uint8_t rep_payload[] = {3 /* svc_idx */, static_cast<uint8_t>(ServiceStatus::OK), 0x11, 0x22};
  can.push_rx(make_extended_frame(rep_eid, rep_payload, 4));

  master.poll();
  EXPECT_TRUE(service_res);
}
