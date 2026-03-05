/// 統合テスト: protoc-gen-protocan 生成コード × protocan_device ランタイム
///
/// カバレッジ:
///   Part 1 — メッセージ encode/decode ラウンドトリップ
///   Part 2 — 生成 Node の直接 API (Device なし)
///   Part 3 — Device + 生成 Node の CAN 経由フルパス

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "bldc_motor.hpp"
#include "protocan/can_frame.hpp"
#include "protocan/types.hpp"
#include "protocan_device/device.hpp"
#include "protocan_device/node_base.hpp"

namespace bm = protocan::bldc_motor;

using namespace protocan;
using namespace protocan::device;

// ════════════════════════════════════════════════════════════════
// 共通モック / ヘルパー
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

  /// Extended-ID フレームの中から fc で絞り込んで最初の 1 件を返す
  const CanFrame * find_ext(FunctionCode fc) const
  {
    for (const auto & f : sent) {
      if (!f.is_extended) continue;
      if (decode_extended_id(f.id).function_code == fc) return &f;
    }
    return nullptr;
  }

  /// Standard-ID (PDO) フレームで pdo_id に一致する最初の 1 件
  const CanFrame * find_pdo(uint16_t pdo_id) const
  {
    for (const auto & f : sent) {
      if (f.is_extended) continue;
      if ((f.id & 0x7FFu) == pdo_id) return &f;
    }
    return nullptr;
  }
};

static uint32_t g_time_ms = 0;
static uint32_t get_time() { return g_time_ms; }
static uint32_t rand_fixed(uint32_t min, uint32_t) { return min; }

// NMT コマンド用フレーム生成ヘルパー
static CanFrame nmt_cmd(uint8_t dst_dev, NmtCommand cmd)
{
  ExtendedId eid{};
  eid.function_code = FunctionCode::NMT;
  eid.src_dev       = kMasterDeviceId;
  eid.src_node      = 0;
  eid.dst_dev       = dst_dev;
  eid.dst_node      = kBroadcastNodeId;
  eid.context       = static_cast<uint8_t>(cmd);
  return make_extended_frame(eid, nullptr, 0);
}

// ════════════════════════════════════════════════════════════════
// Part 1 — メッセージ encode/decode ラウンドトリップ
// ════════════════════════════════════════════════════════════════

TEST(MessageTest, PackedSizeConstants)
{
  static_assert(bm::MotorStatus::PACKED_SIZE == 13, "3*float(4B) + uint8(1B) = 13");
  static_assert(bm::TwistCommand::PACKED_SIZE == 8,  "2*float(4B) = 8");
  static_assert(bm::SetEnableRequest::PACKED_SIZE == 1,  "bool(1B) = 1");
  static_assert(bm::SetEnableResponse::PACKED_SIZE == 1, "bool(1B) = 1");
}

TEST(MessageTest, MotorStatusRoundtrip)
{
  bm::MotorStatus orig{1.5f, -2.5f, 30.125f, 0xABu};
  uint8_t         buf[bm::MotorStatus::PACKED_SIZE];
  orig.encode(buf);

  auto dec = bm::MotorStatus::decode(buf);
  EXPECT_FLOAT_EQ(dec.current_a,     orig.current_a);
  EXPECT_FLOAT_EQ(dec.velocity_rps,  orig.velocity_rps);
  EXPECT_FLOAT_EQ(dec.temperature_c, orig.temperature_c);
  EXPECT_EQ(dec.error_flags, orig.error_flags);
}

TEST(MessageTest, MotorStatusEncodeIsLittleEndian)
{
  // 1.0f in IEEE 754 LE = 0x00 0x00 0x80 0x3F
  bm::MotorStatus s{1.0f, 0.0f, 0.0f, 0};
  uint8_t         buf[bm::MotorStatus::PACKED_SIZE];
  s.encode(buf);

  EXPECT_EQ(buf[0], 0x00u);
  EXPECT_EQ(buf[1], 0x00u);
  EXPECT_EQ(buf[2], 0x80u);
  EXPECT_EQ(buf[3], 0x3Fu);
}

TEST(MessageTest, MotorStatusErrorFlagsRange)
{
  // error_flags は proto: uint32 size=1 → uint8_t。上位バイトは失われる
  bm::MotorStatus s{0.0f, 0.0f, 0.0f, 0xFFu};
  uint8_t         buf[bm::MotorStatus::PACKED_SIZE];
  s.encode(buf);

  EXPECT_EQ(buf[12], 0xFFu);

  auto dec = bm::MotorStatus::decode(buf);
  EXPECT_EQ(dec.error_flags, 0xFFu);
}

TEST(MessageTest, TwistCommandRoundtrip)
{
  bm::TwistCommand orig{0.75f, -3.14159f};
  uint8_t          buf[bm::TwistCommand::PACKED_SIZE];
  orig.encode(buf);

  auto dec = bm::TwistCommand::decode(buf);
  EXPECT_FLOAT_EQ(dec.linear_x,  orig.linear_x);
  EXPECT_FLOAT_EQ(dec.angular_z, orig.angular_z);
}

TEST(MessageTest, SetEnableRequestRoundtrip)
{
  for (bool val : {true, false}) {
    bm::SetEnableRequest orig{val};
    uint8_t              buf[bm::SetEnableRequest::PACKED_SIZE];
    orig.encode(buf);

    EXPECT_EQ(buf[0], val ? 1u : 0u);

    auto dec = bm::SetEnableRequest::decode(buf);
    EXPECT_EQ(dec.enable, val);
  }
}

TEST(MessageTest, SetEnableResponseRoundtrip)
{
  bm::SetEnableResponse orig{true};
  uint8_t               buf[bm::SetEnableResponse::PACKED_SIZE];
  orig.encode(buf);
  auto dec = bm::SetEnableResponse::decode(buf);
  EXPECT_TRUE(dec.success);
}

// ════════════════════════════════════════════════════════════════
// Part 2 — 生成 Node の直接 API (Device なし)
// ════════════════════════════════════════════════════════════════

class NodeApiTest : public ::testing::Test
{
protected:
  bm::Node node{2, "test_motor"};
};

// ── スキーマ定数 ──────────────────────────────────────────────

TEST_F(NodeApiTest, SchemaHashIsNonZero)
{
  EXPECT_NE(bm::SCHEMA_HASH, 0u);
  // Node コンストラクタが NodeBase に SCHEMA_HASH を渡す
  EXPECT_EQ(node.schema_hash(), bm::SCHEMA_HASH);
}

TEST_F(NodeApiTest, DescriptorBlobIsNonEmpty)
{
  EXPECT_GT(bm::DESCRIPTOR_BLOB_SIZE, 0u);
  EXPECT_EQ(node.descriptor_blob_size(), bm::DESCRIPTOR_BLOB_SIZE);
  // 先頭バイトが 0 でないこと (protobuf フィールドタグ)
  EXPECT_NE(bm::DESCRIPTOR_BLOB[0], 0u);
}

TEST_F(NodeApiTest, MaxPdoEntryConstants)
{
  EXPECT_EQ(bm::MAX_PDO_TX_ENTRIES, 4u);  // MotorStatus: 4 fields
  EXPECT_EQ(bm::MAX_PDO_RX_ENTRIES, 2u);  // TwistCommand: 2 fields
}

// ── fill_pdo_tx ───────────────────────────────────────────────

TEST_F(NodeApiTest, FillPdoTxReturnsZeroWithNoEntry)
{
  uint8_t buf[64] = {};
  EXPECT_EQ(node.fill_pdo_tx(0x100, buf, 64), 0u);
}

TEST_F(NodeApiTest, FillPdoTxEncodesMotorStatus)
{
  // MotorStatus: current_a(f=0,off=0,sz=4), velocity_rps(f=1,off=4,sz=4),
  //              temperature_c(f=2,off=8,sz=4), error_flags(f=3,off=12,sz=1)
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 0, 0,  4, 100, 0});
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 1, 4,  4, 100, 0});
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 2, 8,  4, 100, 0});
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 3, 12, 1, 100, 0});

  node.status_buffer() = {1.5f, -2.5f, 30.0f, 0x42u};

  uint8_t buf[64] = {};
  uint8_t len     = node.fill_pdo_tx(0x100, buf, 64);

  ASSERT_EQ(len, static_cast<uint8_t>(bm::MotorStatus::PACKED_SIZE));

  auto dec = bm::MotorStatus::decode(buf);
  EXPECT_FLOAT_EQ(dec.current_a,     1.5f);
  EXPECT_FLOAT_EQ(dec.velocity_rps,  -2.5f);
  EXPECT_FLOAT_EQ(dec.temperature_c, 30.0f);
  EXPECT_EQ(dec.error_flags, 0x42u);
}

TEST_F(NodeApiTest, FillPdoTxReturnsZeroIfBufTooSmall)
{
  // current_a: offset=0, size=4 → offset+size=4 > max_len=3 → skip → len=0
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 0, 0, 4, 100, 0});

  uint8_t buf[4] = {};
  EXPECT_EQ(node.fill_pdo_tx(0x100, buf, 3), 0u);
}

TEST_F(NodeApiTest, FillPdoTxIgnoresWrongPdoId)
{
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 0, 0, 4, 100, 0});

  uint8_t buf[64] = {};
  // 違う pdo_id → 0
  EXPECT_EQ(node.fill_pdo_tx(0x200, buf, 64), 0u);
}

// ── on_pdo_rx ────────────────────────────────────────────────

TEST_F(NodeApiTest, OnPdoRxDeliversToCallback)
{
  // TwistCommand: linear_x(f=0,off=0,sz=4), angular_z(f=1,off=4,sz=4)
  node.add_pdo_rx(PdoRxEntry{0x200, 0, 0, 0, 4});
  node.add_pdo_rx(PdoRxEntry{0x200, 0, 1, 4, 4});

  struct Ctx {
    bool             called;
    bm::TwistCommand received;
  } ctx{};

  node.on_cmd_vel(
    [](const bm::TwistCommand & cmd, void * p) {
      auto & c   = *static_cast<Ctx *>(p);
      c.called   = true;
      c.received = cmd;
    },
    &ctx);

  bm::TwistCommand cmd{0.8f, -1.2f};
  uint8_t          buf[bm::TwistCommand::PACKED_SIZE];
  cmd.encode(buf);

  node.on_pdo_rx(0x200, buf, bm::TwistCommand::PACKED_SIZE);

  ASSERT_TRUE(ctx.called);
  EXPECT_FLOAT_EQ(ctx.received.linear_x,  0.8f);
  EXPECT_FLOAT_EQ(ctx.received.angular_z, -1.2f);
}

TEST_F(NodeApiTest, OnPdoRxNoCallbackRegistered)
{
  node.add_pdo_rx(PdoRxEntry{0x200, 0, 0, 0, 4});
  node.add_pdo_rx(PdoRxEntry{0x200, 0, 1, 4, 4});

  // コールバック未登録 → クラッシュしない
  uint8_t buf[8] = {};
  EXPECT_NO_FATAL_FAILURE(node.on_pdo_rx(0x200, buf, 8));
}

TEST_F(NodeApiTest, OnPdoRxSkipsIfTooShort)
{
  // linear_x: offset=0, size=4 → offset+size=4 > len=3 → skip → callback not called
  node.add_pdo_rx(PdoRxEntry{0x200, 0, 0, 0, 4});

  bool called = false;
  node.on_cmd_vel(
    [](const bm::TwistCommand &, void * p) { *static_cast<bool *>(p) = true; },
    &called);

  uint8_t buf[4] = {};
  node.on_pdo_rx(0x200, buf, 3);
  EXPECT_FALSE(called);
}

// ── on_service_req ────────────────────────────────────────────

TEST_F(NodeApiTest, ServiceCallbackInvoked)
{
  struct Ctx {
    bool enable_seen;
    bool success_returned;
  } ctx{};

  node.on_set_enable(
    [](const bm::SetEnableRequest & req, bm::SetEnableResponse & res, void * p) -> Status {
      auto & c          = *static_cast<Ctx *>(p);
      c.enable_seen     = req.enable;
      res.success       = req.enable;
      c.success_returned = req.enable;
      return Status::OK;
    },
    &ctx);

  bm::SetEnableRequest req{true};
  uint8_t              req_buf[bm::SetEnableRequest::PACKED_SIZE];
  req.encode(req_buf);

  uint8_t res_buf[62] = {};
  uint8_t res_size    = 0;
  Status  s           = node.on_service_req(0, req_buf, sizeof(req_buf), res_buf, res_size);

  EXPECT_EQ(s, Status::OK);
  EXPECT_TRUE(ctx.enable_seen);
  EXPECT_EQ(res_size, static_cast<uint8_t>(bm::SetEnableResponse::PACKED_SIZE));

  auto res = bm::SetEnableResponse::decode(res_buf);
  EXPECT_TRUE(res.success);
}

TEST_F(NodeApiTest, ServiceReturnsNotFoundIfNoCallback)
{
  uint8_t req_buf[1] = {1};
  uint8_t res_buf[62] = {};
  uint8_t res_size    = 0;
  Status  s           = node.on_service_req(0, req_buf, 1, res_buf, res_size);
  EXPECT_EQ(s, Status::NOT_FOUND);
}

TEST_F(NodeApiTest, ServiceReturnsInvalidArgIfReqTooShort)
{
  node.on_set_enable(
    [](const bm::SetEnableRequest &, bm::SetEnableResponse &, void *) { return Status::OK; },
    nullptr);

  uint8_t res_buf[62] = {};
  uint8_t res_size    = 0;
  // rsz=0 < PACKED_SIZE=1 → INVALID_ARGUMENT
  Status s = node.on_service_req(0, nullptr, 0, res_buf, res_size);
  EXPECT_EQ(s, Status::INVALID_ARGUMENT);
}

TEST_F(NodeApiTest, ServiceUnknownIndexReturnsNotFound)
{
  uint8_t req_buf[1] = {};
  uint8_t res_buf[62] = {};
  uint8_t res_size    = 0;
  EXPECT_EQ(node.on_service_req(99, req_buf, 1, res_buf, res_size), Status::NOT_FOUND);
}

// ── on_param_read / on_param_write ───────────────────────────

TEST_F(NodeApiTest, ParamReadCallsGetCallback)
{
  static float stored_kp = 2.5f;

  node.on_get_pid_gains(
    [](uint8_t * out, uint8_t & size, void *) -> Status {
      uint32_t tmp;
      std::memcpy(&tmp, &stored_kp, 4);
      out[0] = tmp;
      out[1] = tmp >> 8;
      out[2] = tmp >> 16;
      out[3] = tmp >> 24;
      size   = 4;
      return Status::OK;
    },
    nullptr);

  uint8_t out[62]    = {};
  uint8_t out_size   = 0;
  Status  s          = node.on_param_read(0, out, out_size);

  EXPECT_EQ(s, Status::OK);
  EXPECT_EQ(out_size, 4u);

  // Decode the LE float
  uint32_t tmp =
    static_cast<uint32_t>(out[0]) | (static_cast<uint32_t>(out[1]) << 8) |
    (static_cast<uint32_t>(out[2]) << 16) | (static_cast<uint32_t>(out[3]) << 24);
  float kp_decoded;
  std::memcpy(&kp_decoded, &tmp, 4);
  EXPECT_FLOAT_EQ(kp_decoded, stored_kp);
}

TEST_F(NodeApiTest, ParamReadNoCallbackReturnsNotFound)
{
  uint8_t out[62] = {};
  uint8_t out_size = 0;
  EXPECT_EQ(node.on_param_read(0, out, out_size), Status::NOT_FOUND);
}

TEST_F(NodeApiTest, ParamWriteCallsSetCallback)
{
  static float written_kp = 0.0f;

  node.on_set_pid_gains(
    [](const uint8_t * data, uint8_t size, void *) -> Status {
      if (size < 4) return Status::INVALID_ARGUMENT;
      uint32_t tmp =
        static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
        (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
      std::memcpy(&written_kp, &tmp, 4);
      return Status::OK;
    },
    nullptr);

  float   kp = 3.14f;
  uint8_t data[4];
  uint32_t tmp;
  std::memcpy(&tmp, &kp, 4);
  data[0] = tmp;
  data[1] = tmp >> 8;
  data[2] = tmp >> 16;
  data[3] = tmp >> 24;

  Status s = node.on_param_write(0, data, 4);

  EXPECT_EQ(s, Status::OK);
  EXPECT_FLOAT_EQ(written_kp, kp);
}

TEST_F(NodeApiTest, ParamWriteNoCallbackReturnsNotFound)
{
  uint8_t data[4] = {};
  EXPECT_EQ(node.on_param_write(0, data, 4), Status::NOT_FOUND);
}

// ── publish_status ───────────────────────────────────────────

TEST_F(NodeApiTest, PublishStatusWithoutSendFnReturnsNotFound)
{
  // send_pdo_fn_ は nullptr (add_node 未呼び出し)
  EXPECT_EQ(node.publish_status(), Status::NOT_FOUND);
}

TEST_F(NodeApiTest, PublishStatusWithoutTxEntryReturnsNotFound)
{
  // 手動で send_pdo_fn を注入する
  struct SentPdo {
    uint16_t       pdo_id;
    std::vector<uint8_t> data;
  } sent;

  node.set_send_pdo(
    [](uint16_t id, const uint8_t * d, uint8_t len, void * ctx) -> Status {
      auto & s  = *static_cast<SentPdo *>(ctx);
      s.pdo_id  = id;
      s.data.assign(d, d + len);
      return Status::OK;
    },
    &sent);

  // TX エントリ未登録 → NOT_FOUND
  EXPECT_EQ(node.publish_status(), Status::NOT_FOUND);
}

TEST_F(NodeApiTest, PublishStatusSendsEncodedFrame)
{
  struct SentPdo {
    uint16_t             pdo_id = 0;
    std::vector<uint8_t> data;
  } sent;

  node.set_send_pdo(
    [](uint16_t id, const uint8_t * d, uint8_t len, void * ctx) -> Status {
      auto & s  = *static_cast<SentPdo *>(ctx);
      s.pdo_id  = id;
      s.data.assign(d, d + len);
      return Status::OK;
    },
    &sent);

  // TX エントリ登録 (pdo_id=0x100, topic_index=0, 全フィールド)
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 0, 0,  4, 100, 0});
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 1, 4,  4, 100, 0});
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 2, 8,  4, 100, 0});
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 3, 12, 1, 100, 0});

  node.status_buffer() = {2.0f, 3.0f, 25.0f, 0x01u};

  EXPECT_EQ(node.publish_status(), Status::OK);
  EXPECT_EQ(sent.pdo_id, 0x100u);
  ASSERT_EQ(sent.data.size(), bm::MotorStatus::PACKED_SIZE);

  auto dec = bm::MotorStatus::decode(sent.data.data());
  EXPECT_FLOAT_EQ(dec.current_a,     2.0f);
  EXPECT_FLOAT_EQ(dec.velocity_rps,  3.0f);
  EXPECT_FLOAT_EQ(dec.temperature_c, 25.0f);
  EXPECT_EQ(dec.error_flags, 0x01u);
}

// ════════════════════════════════════════════════════════════════
// Part 3 — Device + 生成 Node の CAN 経由フルパス
// ════════════════════════════════════════════════════════════════

class DeviceIntegrationTest : public ::testing::Test
{
protected:
  MockCan  can;
  bm::Node node{2, "motor1"};

  void SetUp() override
  {
    g_time_ms = 0;
    can.clear();
  }

  Device make_device() { return Device(3, &can, get_time, rand_fixed); }

  /// PREOP listen-only → OPERATIONAL まで一気に進める
  void boot_to_operational(Device & dev)
  {
    dev.add_node(node);
    dev.start();
    g_time_ms = 101;
    dev.poll();
    can.clear();

    can.push_rx(nmt_cmd(kBroadcastDeviceId, NmtCommand::START));
    dev.poll();
    can.clear();

    ASSERT_EQ(dev.state(), DeviceState::OPERATIONAL);
  }

  /// PARAM READ リクエストフレーム生成 (dst_node=2)
  CanFrame make_param_read_req(uint8_t param_idx)
  {
    ExtendedId eid{};
    eid.function_code = FunctionCode::PARAM;
    eid.src_dev       = kMasterDeviceId;
    eid.src_node      = 0;
    eid.dst_dev       = 3;
    eid.dst_node      = 2;
    eid.context       = static_cast<uint8_t>(ParamCommand::READ);
    uint8_t payload[1] = {param_idx};
    return make_extended_frame(eid, payload, 1);
  }

  /// PARAM WRITE リクエストフレーム生成 (dst_node=2)
  CanFrame make_param_write_req(uint8_t param_idx, const uint8_t * data, uint8_t size)
  {
    ExtendedId eid{};
    eid.function_code = FunctionCode::PARAM;
    eid.src_dev       = kMasterDeviceId;
    eid.src_node      = 0;
    eid.dst_dev       = 3;
    eid.dst_node      = 2;
    eid.context       = static_cast<uint8_t>(ParamCommand::WRITE);
    uint8_t payload[64] = {param_idx};
    std::memcpy(payload + 1, data, size);
    return make_extended_frame(eid, payload, static_cast<uint8_t>(1 + size));
  }

  /// SERVICE リクエストフレーム生成 (dst_node=2)
  CanFrame make_service_req(uint8_t svc_idx, const uint8_t * req_data, uint8_t req_size)
  {
    ExtendedId eid{};
    eid.function_code = FunctionCode::SERVICE;
    eid.src_dev       = kMasterDeviceId;
    eid.src_node      = 0;
    eid.dst_dev       = 3;
    eid.dst_node      = 2;
    eid.context       = 1;  // seq_id
    uint8_t payload[64] = {svc_idx};
    if (req_data && req_size > 0) std::memcpy(payload + 1, req_data, req_size);
    return make_extended_frame(eid, payload, static_cast<uint8_t>(1 + req_size));
  }
};

// ── add_node がトランポリンを注入する ─────────────────────────

TEST_F(DeviceIntegrationTest, AddNodeInjectsSendPdoTrampoline)
{
  Device dev = make_device();
  dev.add_node(node);
  dev.start();
  g_time_ms = 101;
  dev.poll();

  // TX エントリを直接登録し publish_status() を呼ぶ
  // → Device::send_pdo_trampoline 経由で CAN に送信される
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 0, 0,  4, 0, 0});
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 1, 4,  4, 0, 0});
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 2, 8,  4, 0, 0});
  node.add_pdo_tx(PdoTxEntry{0x100, 0, 3, 12, 1, 0, 0});
  node.status_buffer() = {5.0f, 0.0f, 0.0f, 0};

  Status s = node.publish_status();
  EXPECT_EQ(s, Status::OK);

  const CanFrame * pdo = can.find_pdo(0x100);
  ASSERT_NE(pdo, nullptr);
  EXPECT_EQ(pdo->dlc, static_cast<uint8_t>(bm::MotorStatus::PACKED_SIZE));

  auto dec = bm::MotorStatus::decode(pdo->data.data());
  EXPECT_FLOAT_EQ(dec.current_a, 5.0f);
}

// ── イベント駆動 publish (publish_status) ────────────────────

TEST_F(DeviceIntegrationTest, PublishStatusSendsCorrectPdoViaCan)
{
  Device dev = make_device();
  boot_to_operational(dev);

  // PDO_CFG で TX エントリを直接追加
  node.add_pdo_tx(PdoTxEntry{0x101, 0, 0, 0,  4, 0, 0});
  node.add_pdo_tx(PdoTxEntry{0x101, 0, 1, 4,  4, 0, 0});
  node.add_pdo_tx(PdoTxEntry{0x101, 0, 2, 8,  4, 0, 0});
  node.add_pdo_tx(PdoTxEntry{0x101, 0, 3, 12, 1, 0, 0});

  node.status_buffer() = {1.0f, 2.0f, 40.0f, 0x03u};

  EXPECT_EQ(node.publish_status(), Status::OK);

  const CanFrame * pdo = can.find_pdo(0x101);
  ASSERT_NE(pdo, nullptr);
  EXPECT_EQ(pdo->dlc, static_cast<uint8_t>(bm::MotorStatus::PACKED_SIZE));

  auto dec = bm::MotorStatus::decode(pdo->data.data());
  EXPECT_FLOAT_EQ(dec.current_a,     1.0f);
  EXPECT_FLOAT_EQ(dec.velocity_rps,  2.0f);
  EXPECT_FLOAT_EQ(dec.temperature_c, 40.0f);
  EXPECT_EQ(dec.error_flags, 0x03u);
}

// ── 定期 PDO TX (poll ループ) ─────────────────────────────────

TEST_F(DeviceIntegrationTest, PeriodicPdoTxSentInOperational)
{
  Device dev = make_device();
  boot_to_operational(dev);

  // period_ms=50 の TX エントリを追加
  node.add_pdo_tx(PdoTxEntry{0x102, 0, 0, 0,  4, 50, 0});
  node.add_pdo_tx(PdoTxEntry{0x102, 0, 1, 4,  4, 50, 0});
  node.add_pdo_tx(PdoTxEntry{0x102, 0, 2, 8,  4, 50, 0});
  node.add_pdo_tx(PdoTxEntry{0x102, 0, 3, 12, 1, 50, 0});
  node.status_buffer() = {9.0f, 0.0f, 0.0f, 0};

  // 50ms 経過させて poll() → 定期送信が発火する
  g_time_ms = 200;
  dev.poll();

  const CanFrame * pdo = can.find_pdo(0x102);
  ASSERT_NE(pdo, nullptr);

  auto dec = bm::MotorStatus::decode(pdo->data.data());
  EXPECT_FLOAT_EQ(dec.current_a, 9.0f);
}

// ── CAN 経由 PDO RX → コールバック ───────────────────────────

TEST_F(DeviceIntegrationTest, PdoRxViaCanDeliveredToCallback)
{
  Device dev = make_device();
  boot_to_operational(dev);

  // RX エントリを追加 (pdo_id=0x200, topic_index=0=cmd_vel)
  // TwistCommand: linear_x(f=0,off=0,sz=4), angular_z(f=1,off=4,sz=4)
  node.add_pdo_rx(PdoRxEntry{0x200, 0, 0, 0, 4});
  node.add_pdo_rx(PdoRxEntry{0x200, 0, 1, 4, 4});

  struct Ctx {
    bool             called = false;
    bm::TwistCommand received{};
  } ctx;

  node.on_cmd_vel(
    [](const bm::TwistCommand & cmd, void * p) {
      auto & c   = *static_cast<Ctx *>(p);
      c.called   = true;
      c.received = cmd;
    },
    &ctx);

  // TwistCommand を PDO フレームとして送信
  bm::TwistCommand cmd{1.5f, -0.7f};
  uint8_t          buf[bm::TwistCommand::PACKED_SIZE];
  cmd.encode(buf);

  CanFrame pdo_frame = make_standard_frame(0x200, buf, bm::TwistCommand::PACKED_SIZE);
  can.push_rx(pdo_frame);
  dev.poll();

  ASSERT_TRUE(ctx.called);
  EXPECT_FLOAT_EQ(ctx.received.linear_x,  1.5f);
  EXPECT_FLOAT_EQ(ctx.received.angular_z, -0.7f);
}

// ── CAN 経由 SERVICE → コールバック → レスポンス ─────────────

TEST_F(DeviceIntegrationTest, ServiceViaCanInvokesCallbackAndSendsResponse)
{
  Device dev = make_device();
  boot_to_operational(dev);

  struct Ctx {
    bool enable_received = false;
  } ctx;

  node.on_set_enable(
    [](const bm::SetEnableRequest & req, bm::SetEnableResponse & res, void * p) -> Status {
      auto & c          = *static_cast<Ctx *>(p);
      c.enable_received = req.enable;
      res.success       = req.enable;
      return Status::OK;
    },
    &ctx);

  // SetEnableRequest{enable=true} をエンコードしてサービスリクエスト送信
  bm::SetEnableRequest req{true};
  uint8_t              req_buf[bm::SetEnableRequest::PACKED_SIZE];
  req.encode(req_buf);

  can.push_rx(make_service_req(0, req_buf, sizeof(req_buf)));
  dev.poll();

  EXPECT_TRUE(ctx.enable_received);

  // SERVICE レスポンスフレームが送信されていること
  const CanFrame * res_frame = can.find_ext(FunctionCode::SERVICE);
  ASSERT_NE(res_frame, nullptr);

  ExtendedId res_eid = decode_extended_id(res_frame->id);
  EXPECT_EQ(res_eid.function_code, FunctionCode::SERVICE);
  EXPECT_EQ(res_eid.src_dev, 3u);
  EXPECT_EQ(res_frame->data[0], 0u);  // svc_idx
  EXPECT_EQ(res_frame->data[1], static_cast<uint8_t>(ServiceStatus::OK));

  // レスポンスを decode → success=true
  auto res = bm::SetEnableResponse::decode(res_frame->data.data() + 2);
  EXPECT_TRUE(res.success);
}

// ── CAN 経由 PARAM READ → get コールバック ───────────────────

TEST_F(DeviceIntegrationTest, ParamReadViaCanInvokesGetCallback)
{
  Device dev = make_device();
  boot_to_operational(dev);

  static float stored_kp = 1.25f;

  node.on_get_pid_gains(
    [](uint8_t * out, uint8_t & size, void *) -> Status {
      uint32_t tmp;
      std::memcpy(&tmp, &stored_kp, 4);
      out[0] = tmp;
      out[1] = tmp >> 8;
      out[2] = tmp >> 16;
      out[3] = tmp >> 24;
      size   = 4;
      return Status::OK;
    },
    nullptr);

  can.push_rx(make_param_read_req(0));
  dev.poll();

  // PARAM READ_RES が送信されていること
  const CanFrame * res_frame = can.find_ext(FunctionCode::PARAM);
  ASSERT_NE(res_frame, nullptr);

  ExtendedId res_eid = decode_extended_id(res_frame->id);
  EXPECT_EQ(res_eid.context, static_cast<uint8_t>(ParamCommand::READ_RES));
  EXPECT_EQ(res_frame->data[0], 0u);  // param_idx
  EXPECT_EQ(res_frame->data[1], static_cast<uint8_t>(ParamStatus::OK));

  // data[2..5] が float(1.25f) のLE
  uint32_t tmp =
    static_cast<uint32_t>(res_frame->data[2]) |
    (static_cast<uint32_t>(res_frame->data[3]) << 8) |
    (static_cast<uint32_t>(res_frame->data[4]) << 16) |
    (static_cast<uint32_t>(res_frame->data[5]) << 24);
  float kp_received;
  std::memcpy(&kp_received, &tmp, 4);
  EXPECT_FLOAT_EQ(kp_received, stored_kp);
}

// ── CAN 経由 PARAM WRITE → set コールバック ──────────────────

TEST_F(DeviceIntegrationTest, ParamWriteViaCanInvokesSetCallback)
{
  Device dev = make_device();
  boot_to_operational(dev);

  static float written_kp = 0.0f;

  node.on_set_pid_gains(
    [](const uint8_t * data, uint8_t size, void *) -> Status {
      if (size < 4) return Status::INVALID_ARGUMENT;
      uint32_t tmp =
        static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
        (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
      std::memcpy(&written_kp, &tmp, 4);
      return Status::OK;
    },
    nullptr);

  // kp=0.5f を LE で書き込み
  float    kp = 0.5f;
  uint32_t tmp;
  std::memcpy(&tmp, &kp, 4);
  uint8_t data[4] = {
    static_cast<uint8_t>(tmp),       static_cast<uint8_t>(tmp >> 8),
    static_cast<uint8_t>(tmp >> 16), static_cast<uint8_t>(tmp >> 24)};

  can.push_rx(make_param_write_req(0, data, 4));
  dev.poll();

  EXPECT_FLOAT_EQ(written_kp, kp);

  // PARAM WRITE_RES が送信されていること
  const CanFrame * res_frame = can.find_ext(FunctionCode::PARAM);
  ASSERT_NE(res_frame, nullptr);

  ExtendedId res_eid = decode_extended_id(res_frame->id);
  EXPECT_EQ(res_eid.context, static_cast<uint8_t>(ParamCommand::WRITE_RES));
  EXPECT_EQ(res_frame->data[1], static_cast<uint8_t>(ParamStatus::OK));
}

// ── DISC → BULK でディスクリプタを取得する ───────────────────

TEST_F(DeviceIntegrationTest, DiscRequestSendsDescriptorBlob)
{
  Device dev = make_device();
  boot_to_operational(dev);

  // DISC リクエスト送信 (dst_node=2)
  ExtendedId disc_eid{};
  disc_eid.function_code = FunctionCode::DISC;
  disc_eid.src_dev       = kMasterDeviceId;
  disc_eid.src_node      = 0;
  disc_eid.dst_dev       = 3;
  disc_eid.dst_node      = 2;
  disc_eid.context       = 0;
  can.push_rx(make_extended_frame(disc_eid, nullptr, 0));
  dev.poll();

  // BULK First Frame が送信されていること
  const CanFrame * bulk_frame = can.find_ext(FunctionCode::BULK);
  ASSERT_NE(bulk_frame, nullptr);
  EXPECT_EQ(bulk_frame->data[0], static_cast<uint8_t>(BulkFrameType::FIRST_FRAME));
  EXPECT_EQ(bulk_frame->data[1], static_cast<uint8_t>(BulkPayloadType::DESCRIPTOR));

  // total_length == DESCRIPTOR_BLOB_SIZE
  uint32_t total =
    static_cast<uint32_t>(bulk_frame->data[2]) |
    (static_cast<uint32_t>(bulk_frame->data[3]) << 8) |
    (static_cast<uint32_t>(bulk_frame->data[4]) << 16) |
    (static_cast<uint32_t>(bulk_frame->data[5]) << 24);
  EXPECT_EQ(total, static_cast<uint32_t>(bm::DESCRIPTOR_BLOB_SIZE));

  // BLOB 先頭バイトがフレームに含まれること
  EXPECT_EQ(bulk_frame->data[6], bm::DESCRIPTOR_BLOB[0]);
}
