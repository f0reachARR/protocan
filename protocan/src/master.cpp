#include "protocan/master.hpp"

#include <algorithm>
#include <cstring>

#include "protocan/descriptor_parser.hpp"
#include "protocan/schema_hash.hpp"

namespace protocan
{

// ── LE ペイロードヘルパー (ローカル) ──

static inline uint16_t read_le16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

static inline uint32_t read_le32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
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

// ════════════════════════════════════════════════════════════════
// Master
// ════════════════════════════════════════════════════════════════

Master::Master(
  ICanInterface & can_if, MasterCallbacks callbacks, MasterAutomationOptions automation)
: can_if_(can_if), callbacks_(std::move(callbacks)), automation_(automation)
{
}

Master::~Master() = default;

// ── メインループ ──

void Master::poll()
{
  auto frame_opt = can_if_.receive();
  if (!frame_opt) return;
  process_frame(*frame_opt);
}

void Master::tick(std::chrono::steady_clock::time_point now)
{
  // タイムアウト検出
  auto timed_out = tracker_.detect_timeouts(heartbeat_timeout_);
  for (uint8_t dev_id : timed_out) {
    if (callbacks_.on_device_timeout) {
      callbacks_.on_device_timeout(dev_id);
    }
  }

  // Service タイムアウト検出
  std::vector<uint8_t> expired_seqs;
  for (auto & [seq_id, pending] : pending_services_) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - pending.sent_at);
    if (elapsed.count() > kServiceTimeoutMs) {
      expired_seqs.push_back(seq_id);
    }
  }
  for (uint8_t seq_id : expired_seqs) {
    pending_services_.erase(seq_id);
  }
}

// ── フレームディスパッチ ──

void Master::process_frame(const CanFrame & frame)
{
  if (is_management_frame(frame)) {
    process_management_frame(frame);
  } else {
    process_pdo_frame(frame);
  }
}

void Master::process_management_frame(const CanFrame & frame)
{
  ExtendedId eid = decode_extended_id(frame.id);

  switch (eid.function_code) {
    case FunctionCode::NMT:
      handle_nmt(eid, frame.data.data(), frame.dlc);
      break;
    case FunctionCode::EMCY:
      handle_emcy(eid, frame.data.data(), frame.dlc);
      break;
    case FunctionCode::DISC:
      handle_disc(eid, frame.data.data(), frame.dlc);
      break;
    case FunctionCode::PARAM:
      handle_param(eid, frame.data.data(), frame.dlc);
      break;
    case FunctionCode::PDO_CFG:
      handle_pdo_cfg(eid, frame.data.data(), frame.dlc);
      break;
    case FunctionCode::BULK:
      handle_bulk(eid, frame.data.data(), frame.dlc);
      break;
    case FunctionCode::SERVICE:
      handle_service(eid, frame.data.data(), frame.dlc);
      break;
    default:
      break;
  }
}

void Master::process_pdo_frame(const CanFrame & frame)
{
  uint16_t pdo_id = static_cast<uint16_t>(frame.id & 0x7FF);
  const uint8_t * data = frame.data.data();
  uint8_t len = frame.dlc;

  // 1. 生データコールバック (常に発火)
  if (callbacks_.on_pdo_received) {
    callbacks_.on_pdo_received(pdo_id, data, len);
  }

  // 2. ディスクリプタベースのデコード (マッピングが登録済みの場合のみ)
  if (!callbacks_.on_pdo_data) return;

  auto mapping_opt = pdo_mgr_.get_mapping(pdo_id);
  if (!mapping_opt) return;

  const auto & mapping = *mapping_opt;

  PdoDecodedData decoded;
  decoded.pdo_id = pdo_id;
  decoded.direction = mapping.direction;
  decoded.raw_data = data;
  decoded.raw_len = len;

  for (const auto & entry : mapping.entries) {
    // エントリの offset + size がフレーム長を超えていたらスキップ
    if (entry.offset + entry.size > len) continue;

    // DeviceTracker の O(1) インデックスで直接フィールド情報を取得
    const auto * topic =
      tracker_.get_topic(entry.device_id, entry.local_node_id, entry.topic_index);
    if (!topic) continue;

    const auto * field = tracker_.get_field(
      entry.device_id, entry.local_node_id, entry.topic_index, entry.field_index);
    if (!field) continue;

    // Packed Binary デコード
    FieldValue value = decode_field(data, entry.offset, field->type, entry.size);

    PdoDecodedField df;
    df.device_id = entry.device_id;
    df.local_node_id = entry.local_node_id;
    df.topic_index = entry.topic_index;
    df.topic_name = topic->name;
    df.field_index = entry.field_index;
    df.field_name = field->name;
    df.value = value;

    decoded.fields.push_back(std::move(df));
  }

  if (!decoded.fields.empty()) {
    callbacks_.on_pdo_data(decoded);
  }
}

// ── NMT ──

void Master::handle_nmt(const ExtendedId & eid, const uint8_t * data, uint8_t len)
{
  // NMT HEARTBEAT (Device → Broadcast)
  // eid.src_dev = device, eid.context = 0 → HEARTBEAT
  if (eid.context != 0) return;  // NMT_CTRL response は通常デバイス側が送らない

  if (len < 6) return;

  Heartbeat hb;
  hb.state = static_cast<DeviceState>(data[0]);
  hb.num_nodes = data[1];
  hb.uptime_ms = read_le32(data + 2);

  // Per-node entries (8 bytes each)
  for (uint8_t i = 0; i < hb.num_nodes; ++i) {
    size_t base = 6 + static_cast<size_t>(i) * 8;
    if (base + 8 > len) break;

    HeartbeatNodeEntry entry;
    entry.schema_hash = read_le32(data + base);
    entry.local_node_id = data[base + 4];
    hb.nodes.push_back(entry);
  }

  bool is_new = !tracker_.get_device(eid.src_dev).has_value();
  tracker_.update_heartbeat(eid.src_dev, hb);

  if (is_new && callbacks_.on_device_discovered) {
    callbacks_.on_device_discovered(eid.src_dev, *tracker_.get_device(eid.src_dev));
  }

  run_automation_for_device(eid.src_dev);
}

// ── EMCY ──

void Master::handle_emcy(const ExtendedId & eid, const uint8_t * data, uint8_t len)
{
  if (len < 2) return;

  EmcyMessage emcy;
  emcy.error_register = read_le16(data);
  if (len > 2) {
    emcy.error_data.assign(data + 2, data + len);
  }

  if (callbacks_.on_emcy_received) {
    callbacks_.on_emcy_received(eid.src_dev, eid.src_node, emcy);
  }
}

// ── DISC ──

void Master::handle_disc(const ExtendedId & eid, const uint8_t * data, uint8_t len)
{
  // マスターはDISC_GET_DESCRIPTORの送信側。
  // デバイスからの応答はBULK転送で来るため、ここでは特に処理しない。
  (void)eid;
  (void)data;
  (void)len;
}

// ── PARAM ──

void Master::handle_param(const ExtendedId & eid, const uint8_t * data, uint8_t len)
{
  // マスターがリクエストを送り、デバイスからレスポンスが返ってくる
  // ctx: READ_RES (0x81) or WRITE_RES (0x82)
  auto cmd = static_cast<ParamCommand>(eid.context);
  if (cmd != ParamCommand::READ_RES && cmd != ParamCommand::WRITE_RES) {
    return;
  }

  if (len < 2) return;

  uint8_t param_index = data[0];
  auto status = static_cast<ParamStatus>(data[1]);
  const uint8_t * value_data = (len > 2) ? (data + 2) : nullptr;
  size_t value_len = (len > 2) ? (len - 2) : 0;

  if (callbacks_.on_param_response) {
    callbacks_.on_param_response(
      eid.src_dev, eid.src_node, cmd, param_index, status, value_data, value_len);
  }
}

// ── PDO_CFG ──

void Master::handle_pdo_cfg(const ExtendedId & eid, const uint8_t * data, uint8_t len)
{
  // マスターからの送信に対するデバイスの ACK
  if (len < 3) return;

  PdoCfgAck ack;
  ack.pdo_id = read_le16(data);
  ack.status = static_cast<PdoCfgStatus>(data[2]);

  // ACK 処理（必要に応じてコールバック追加可能）
  (void)ack;
}

// ── BULK ──

void Master::handle_bulk(const ExtendedId & eid, const uint8_t * data, uint8_t len)
{
  if (len < 1) return;

  auto frame_type = static_cast<BulkFrameType>(data[0]);
  BulkRxKey key{eid.src_dev, eid.src_node, eid.context};

  switch (frame_type) {
    case BulkFrameType::FIRST_FRAME: {
      // 新しい BULK 受信セッションを開始
      auto send_fc = [this](const CanFrame & fc) -> Status { return send_frame(fc); };
      auto [it, _] = bulk_receivers_.emplace(key, BulkReceiver(send_fc));
      it->second.on_first_frame(eid, data, len);

      if (it->second.state() == BulkRxState::COMPLETE) {
        on_bulk_complete(eid, it->second);
        bulk_receivers_.erase(key);
      }
      break;
    }
    case BulkFrameType::CONSECUTIVE_FRAME: {
      auto it = bulk_receivers_.find(key);
      if (it == bulk_receivers_.end()) break;

      it->second.on_consecutive_frame(eid, data, len);

      if (it->second.state() == BulkRxState::COMPLETE) {
        on_bulk_complete(eid, it->second);
        bulk_receivers_.erase(key);
      } else if (it->second.state() == BulkRxState::ERROR) {
        bulk_receivers_.erase(key);
      }
      break;
    }
    case BulkFrameType::FLOW_CONTROL:
      // マスターが BULK 送信する場合のFC受信。現在のところ DISC 応答は受信のみ。
      break;
  }
}

// ── BULK 完了 ──

void Master::on_bulk_complete(const ExtendedId & eid, const BulkReceiver & receiver)
{
  if (receiver.payload_type() == BulkPayloadType::DESCRIPTOR) {
    // ディスクリプタ Blob をパース
    ParsedDescriptor desc;
    if (parse_descriptor(receiver.data().data(), receiver.data().size(), desc)) {
      // キャッシュに格納 (衝突チェック)
      auto cached = tracker_.get_cached_descriptor(desc.schema_hash);
      if (cached.has_value()) {
        // 既にキャッシュがある: バイナリ内容が異なればハッシュ衝突
        // → キャッシュを無効化して個別管理
        // (簡易実装: ここでは node_type_name で比較)
        if (cached->node_type_name != desc.node_type_name) {
          tracker_.invalidate_cache(desc.schema_hash);
        }
      }
      tracker_.cache_descriptor(desc.schema_hash, desc);

      run_automation_for_device(eid.src_dev);

      if (callbacks_.on_descriptor_received) {
        bool notified = false;
        if (auto info_opt = tracker_.get_device(eid.src_dev)) {
          for (const auto & node : info_opt->nodes) {
            if (node.schema_hash != desc.schema_hash) continue;
            callbacks_.on_descriptor_received(eid.src_dev, node.local_node_id, desc);
            notified = true;
          }
        }
        if (!notified) {
          callbacks_.on_descriptor_received(eid.src_dev, eid.src_node, desc);
        }
      }
    }
  } else if (receiver.payload_type() == BulkPayloadType::SERVICE_RES) {
    // BULK 経由の Service Response
    const auto & data = receiver.data();
    if (data.size() >= 2) {
      uint8_t service_index = data[0];
      auto status = static_cast<ServiceStatus>(data[1]);
      const uint8_t * resp_data = (data.size() > 2) ? (data.data() + 2) : nullptr;
      size_t resp_len = (data.size() > 2) ? (data.size() - 2) : 0;

      if (callbacks_.on_service_response) {
        callbacks_.on_service_response(
          eid.src_dev, eid.src_node, service_index, status, resp_data, resp_len);
      }
    }
  }
}

// ── SERVICE ──

void Master::handle_service(const ExtendedId & eid, const uint8_t * data, uint8_t len)
{
  // SERVICE_RES (Device → Master)
  if (len < 2) return;

  uint8_t service_index = data[0];
  auto status = static_cast<ServiceStatus>(data[1]);
  const uint8_t * resp_data = (len > 2) ? (data + 2) : nullptr;
  size_t resp_len = (len > 2) ? (len - 2) : 0;

  // sequence_id を解放
  pending_services_.erase(eid.context);

  if (callbacks_.on_service_response) {
    callbacks_.on_service_response(
      eid.src_dev, eid.src_node, service_index, status, resp_data, resp_len);
  }
}

// ════════════════════════════════════════════════════════════════
// 送信 API
// ════════════════════════════════════════════════════════════════

Status Master::send_nmt_ctrl(uint8_t target_device_id, NmtCommand command)
{
  ExtendedId eid;
  eid.function_code = FunctionCode::NMT;
  eid.src_dev = kMasterDeviceId;
  eid.src_node = 0;
  eid.dst_dev = target_device_id;
  eid.dst_node = kBroadcastNodeId;
  eid.context = static_cast<uint8_t>(command);

  CanFrame frame = make_extended_frame(eid, nullptr, 0);
  return send_frame(frame);
}

Status Master::broadcast_enter_preop()
{
  return send_nmt_ctrl(kBroadcastDeviceId, NmtCommand::ENTER_PREOP);
}

Status Master::send_disc_get_descriptor(uint8_t target_device_id, uint8_t target_node_id)
{
  ExtendedId eid;
  eid.function_code = FunctionCode::DISC;
  eid.src_dev = kMasterDeviceId;
  eid.src_node = 0;
  eid.dst_dev = target_device_id;
  eid.dst_node = target_node_id;
  eid.context = 0;

  CanFrame frame = make_extended_frame(eid, nullptr, 0);
  return send_frame(frame);
}

Status Master::send_param_read(
  uint8_t target_device_id, uint8_t target_node_id, uint8_t param_index)
{
  ExtendedId eid;
  eid.function_code = FunctionCode::PARAM;
  eid.src_dev = kMasterDeviceId;
  eid.src_node = 0;
  eid.dst_dev = target_device_id;
  eid.dst_node = target_node_id;
  eid.context = static_cast<uint8_t>(ParamCommand::READ);

  uint8_t payload[1] = {param_index};
  CanFrame frame = make_extended_frame(eid, payload, 1);
  return send_frame(frame);
}

Status Master::send_param_write(
  uint8_t target_device_id, uint8_t target_node_id, uint8_t param_index, const uint8_t * value,
  uint8_t value_len)
{
  ExtendedId eid;
  eid.function_code = FunctionCode::PARAM;
  eid.src_dev = kMasterDeviceId;
  eid.src_node = 0;
  eid.dst_dev = target_device_id;
  eid.dst_node = target_node_id;
  eid.context = static_cast<uint8_t>(ParamCommand::WRITE);

  uint8_t payload[64] = {};
  payload[0] = param_index;
  uint8_t copy_len = std::min(value_len, static_cast<uint8_t>(63));
  if (value && copy_len > 0) {
    std::memcpy(payload + 1, value, copy_len);
  }

  CanFrame frame = make_extended_frame(eid, payload, static_cast<uint8_t>(1 + copy_len));
  return send_frame(frame);
}

Status Master::send_pdo_cfg(uint8_t target_device_id, const PdoMapping & mapping)
{
  // BEGIN
  {
    ExtendedId eid;
    eid.function_code = FunctionCode::PDO_CFG;
    eid.src_dev = kMasterDeviceId;
    eid.src_node = 0;
    eid.dst_dev = target_device_id;
    eid.dst_node = kBroadcastNodeId;
    eid.context = static_cast<uint8_t>(PdoCfgSequence::BEGIN);

    uint8_t payload[7] = {};
    write_le16(payload, mapping.pdo_id);
    payload[2] = static_cast<uint8_t>(mapping.direction);
    payload[3] = static_cast<uint8_t>(mapping.entries.size());
    write_le16(payload + 4, mapping.period_ms);
    payload[6] = mapping.total_size;

    CanFrame frame = make_extended_frame(eid, payload, 7);
    Status s = send_frame(frame);
    if (s != Status::OK) return s;
  }

  // ENTRY × N
  for (size_t i = 0; i < mapping.entries.size() && i < 0x1E; ++i) {
    const auto & entry = mapping.entries[i];

    ExtendedId eid;
    eid.function_code = FunctionCode::PDO_CFG;
    eid.src_dev = kMasterDeviceId;
    eid.src_node = 0;
    eid.dst_dev = target_device_id;
    eid.dst_node = kBroadcastNodeId;
    eid.context = static_cast<uint8_t>(i + 1);  // 0x01..0x1E

    uint8_t payload[5] = {};
    payload[0] = entry.local_node_id;
    payload[1] = entry.topic_index;
    payload[2] = entry.field_index;
    payload[3] = entry.offset;
    payload[4] = entry.size;

    CanFrame frame = make_extended_frame(eid, payload, 5);
    Status s = send_frame(frame);
    if (s != Status::OK) return s;
  }

  // COMMIT
  {
    ExtendedId eid;
    eid.function_code = FunctionCode::PDO_CFG;
    eid.src_dev = kMasterDeviceId;
    eid.src_node = 0;
    eid.dst_dev = target_device_id;
    eid.dst_node = kBroadcastNodeId;
    eid.context = static_cast<uint8_t>(PdoCfgSequence::COMMIT);

    uint8_t payload[3] = {};
    write_le16(payload, mapping.pdo_id);
    payload[2] = static_cast<uint8_t>(PdoCfgAction::APPLY);

    CanFrame frame = make_extended_frame(eid, payload, 3);
    Status s = send_frame(frame);
    if (s != Status::OK) return s;
  }

  // ローカルにもマッピングを登録
  pdo_mgr_.set_mapping(mapping);

  return Status::OK;
}

Status Master::send_pdo_cfg_delete(uint8_t target_device_id, uint16_t pdo_id)
{
  ExtendedId eid;
  eid.function_code = FunctionCode::PDO_CFG;
  eid.src_dev = kMasterDeviceId;
  eid.src_node = 0;
  eid.dst_dev = target_device_id;
  eid.dst_node = kBroadcastNodeId;
  eid.context = static_cast<uint8_t>(PdoCfgSequence::COMMIT);

  uint8_t payload[3] = {};
  write_le16(payload, pdo_id);
  payload[2] = static_cast<uint8_t>(PdoCfgAction::DELETE);

  CanFrame frame = make_extended_frame(eid, payload, 3);
  Status s = send_frame(frame);
  if (s == Status::OK) {
    pdo_mgr_.remove_mapping(pdo_id);
  }
  return s;
}

std::optional<uint16_t> Master::find_pdo_id(
  uint8_t device_id, uint8_t local_node_id, uint8_t topic_index, PdoCfgDirection direction) const
{
  for (const auto & [pdo_id, mapping] : pdo_mgr_.mappings()) {
    if (mapping.direction != direction) continue;
    for (const auto & entry : mapping.entries) {
      if (
        entry.device_id == device_id && entry.local_node_id == local_node_id &&
        entry.topic_index == topic_index) {
        return pdo_id;
      }
    }
  }
  return std::nullopt;
}

std::optional<uint8_t> Master::send_service_request(
  uint8_t target_device_id, uint8_t target_node_id, uint8_t service_index,
  const uint8_t * request_data, size_t request_len)
{
  // sequence_id を採番
  uint8_t seq_id = next_service_seq_;
  next_service_seq_ = (next_service_seq_ + 1) & 0x1F;  // 5-bit: 0–31

  // 既に使用中の seq_id かチェック
  if (pending_services_.find(seq_id) != pending_services_.end()) {
    return std::nullopt;  // 全スロット使用中
  }

  size_t total_payload = 1 + request_len;  // service_index + request_data

  if (total_payload <= kCanFdMaxPayload) {
    // 単一フレームに収まる → SERVICE フレーム
    ExtendedId eid;
    eid.function_code = FunctionCode::SERVICE;
    eid.src_dev = kMasterDeviceId;
    eid.src_node = 0;
    eid.dst_dev = target_device_id;
    eid.dst_node = target_node_id;
    eid.context = seq_id;

    uint8_t payload[64] = {};
    payload[0] = service_index;
    if (request_data && request_len > 0) {
      std::memcpy(payload + 1, request_data, std::min(request_len, static_cast<size_t>(63)));
    }

    CanFrame frame = make_extended_frame(eid, payload, static_cast<uint8_t>(total_payload));
    Status s = send_frame(frame);
    if (s != Status::OK) return std::nullopt;
  } else {
    // BULK 転送 (spec §5.8: payload_type=1, channel=seq_id)
    ExtendedId eid;
    eid.function_code = FunctionCode::BULK;
    eid.src_dev = kMasterDeviceId;
    eid.src_node = 0;
    eid.dst_dev = target_device_id;
    eid.dst_node = target_node_id;
    eid.context = seq_id;  // channel = seq_id

    // BULK ペイロードは service_index + request_data
    std::vector<uint8_t> bulk_payload(total_payload);
    bulk_payload[0] = service_index;
    if (request_data && request_len > 0) {
      std::memcpy(bulk_payload.data() + 1, request_data, request_len);
    }

    auto sender = BulkSender([this](const CanFrame & f) { return send_frame(f); });
    Status s =
      sender.start(eid, BulkPayloadType::SERVICE_REQ, bulk_payload.data(), bulk_payload.size());
    if (s != Status::OK) return std::nullopt;

    // 残りのフレームを送信 (同期送信)
    while (sender.state() == BulkTxState::SENDING) {
      sender.send_next();
    }
  }

  // pending に登録
  PendingService ps;
  ps.device_id = target_device_id;
  ps.node_id = target_node_id;
  ps.service_index = service_index;
  ps.sent_at = std::chrono::steady_clock::now();
  pending_services_[seq_id] = ps;

  return seq_id;
}

// ── ヘルパー ──

Status Master::send_frame(const CanFrame & frame) { return can_if_.send(frame); }

bool Master::has_all_descriptors(const DeviceInfo & info) const
{
  for (const auto & node : info.nodes) {
    if (!tracker_.is_schema_known(node.schema_hash)) {
      return false;
    }
  }
  return true;
}

bool Master::node_topology_changed(uint8_t device_id, const DeviceInfo & info) const
{
  auto it = auto_device_states_.find(device_id);
  if (it == auto_device_states_.end()) return true;

  auto normalize = [](const std::vector<NodeInfo> & nodes) {
    std::vector<NodeInfo> out = nodes;
    std::sort(out.begin(), out.end(), [](const NodeInfo & a, const NodeInfo & b) {
      if (a.local_node_id != b.local_node_id) return a.local_node_id < b.local_node_id;
      return a.schema_hash < b.schema_hash;
    });
    return out;
  };

  std::vector<NodeInfo> old_nodes = normalize(it->second.configured_nodes);
  std::vector<NodeInfo> new_nodes = normalize(info.nodes);
  if (old_nodes.size() != new_nodes.size()) return true;
  for (size_t i = 0; i < old_nodes.size(); ++i) {
    if (old_nodes[i].local_node_id != new_nodes[i].local_node_id) return true;
    if (old_nodes[i].schema_hash != new_nodes[i].schema_hash) return true;
  }
  return false;
}

Status Master::reconfigure_device(uint8_t device_id, const DeviceInfo & info)
{
  std::vector<NodeConfig> node_configs;
  node_configs.reserve(info.nodes.size());

  for (const auto & node : info.nodes) {
    const ParsedDescriptor * desc = tracker_.get_descriptor_ptr(node.schema_hash);
    if (!desc) return Status::NOT_FOUND;
    node_configs.push_back(NodeConfig{node.local_node_id, desc});
  }

  auto & state = auto_device_states_[device_id];
  for (uint16_t old_pdo_id : state.configured_pdo_ids) {
    send_pdo_cfg_delete(device_id, old_pdo_id);
  }
  state.configured_pdo_ids.clear();

  auto mappings = pdo_mgr_.generate_optimal_mappings(device_id, node_configs);
  for (const auto & mapping : mappings) {
    Status s = send_pdo_cfg(device_id, mapping);
    if (s != Status::OK) return s;
    state.configured_pdo_ids.push_back(mapping.pdo_id);
  }

  state.pdo_configured = true;
  state.configured_nodes = info.nodes;
  return Status::OK;
}

void Master::run_automation_for_device(uint8_t device_id)
{
  if (
    !automation_.auto_request_descriptors && !automation_.auto_configure_pdo &&
    !automation_.auto_manage_state) {
    return;
  }

  auto info_opt = tracker_.get_device(device_id);
  if (!info_opt) return;
  const DeviceInfo & info = *info_opt;

  auto & state = auto_device_states_[device_id];

  if (info.state == DeviceState::PREOP) {
    state.preop_requested = false;
  }
  if (info.state == DeviceState::OPERATIONAL) {
    state.start_requested = true;
  }

  // ノード一覧の取得や再構成のため、必要時は PREOP へ遷移させる
  if (
    automation_.auto_manage_state && info.state == DeviceState::OPERATIONAL &&
    !state.pdo_configured && !state.preop_requested) {
    if (send_nmt_ctrl(device_id, NmtCommand::ENTER_PREOP) == Status::OK) {
      state.preop_requested = true;
    }
    return;
  }

  if (info.state != DeviceState::PREOP) {
    return;
  }

  if (automation_.auto_request_descriptors) {
    for (const auto & node : info.nodes) {
      if (!tracker_.is_schema_known(node.schema_hash)) {
        send_disc_get_descriptor(device_id, node.local_node_id);
      }
    }
  }

  if (!has_all_descriptors(info)) {
    return;
  }

  if (automation_.auto_configure_pdo) {
    if (!state.pdo_configured || node_topology_changed(device_id, info)) {
      if (reconfigure_device(device_id, info) != Status::OK) {
        return;
      }
      state.start_requested = false;
    }
  }

  if (
    automation_.auto_manage_state && state.pdo_configured && !state.start_requested &&
    info.state == DeviceState::PREOP) {
    if (send_nmt_ctrl(device_id, NmtCommand::START) == Status::OK) {
      state.start_requested = true;
    }
  }
}

}  // namespace protocan
