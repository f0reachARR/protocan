#include "protocan_device/device.hpp"

#include <cstring>

#include "protocan/can_frame.hpp"
#include "protocan/types.hpp"

namespace protocan::device
{

// ── LE ペイロードヘルパー ──

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
// Device
// ════════════════════════════════════════════════════════════════

Device::Device(
  uint8_t device_id, ICanInterface * can, GetTimeMsFn get_time_ms, RandMsFn rand_ms)
: device_id_(device_id), can_(can), get_time_ms_(get_time_ms), rand_ms_(rand_ms)
{
  for (uint8_t i = 0; i < kMaxNodes; ++i) {
    nodes_[i] = nullptr;
  }
}

Status Device::add_node(NodeBase & node)
{
  if (node_count_ >= kMaxNodes) {
    return Status::NO_RESOURCE;
  }
  nodes_[node_count_++] = &node;
  node.set_send_pdo(&Device::send_pdo_trampoline, this);
  return Status::OK;
}

void Device::start()
{
  start_ms_ = get_time_ms_();
  listen_only_ = true;
  listen_only_end_ms_ = start_ms_ + rand_ms_(kListenOnlyMinMs, kListenOnlyMaxMs);
  state_ = DeviceState::PREOP;
  last_heartbeat_ms_ = 0;
  initial_hb_count_ = 0;
  bulk_tx_.active = false;
  pdo_cfg_pending_.active = false;
}

void Device::poll()
{
  uint32_t now = get_time_ms_();

  // Listen-Only タイムアウト確認
  if (listen_only_ && now - listen_only_end_ms_ < 0x80000000u) {
    // listen_only_end_ms_ <= now (考慮: overflow safe比較)
    listen_only_ = false;
    initial_hb_count_ = 0;
    last_heartbeat_ms_ = 0;  // 即座にハートビート送信
  }

  // フレーム受信処理
  CanFrame f;
  while (can_->try_receive(f)) {
    handle_frame(f);
  }

  // ハートビート送信
  if (!listen_only_ && state_ != DeviceState::ERROR && state_ != DeviceState::STOPPED) {
    uint32_t interval =
      (state_ == DeviceState::PREOP) ? kHeartbeatIntervalPreopMs : kHeartbeatIntervalOpMs;

    if (now - last_heartbeat_ms_ >= interval) {
      send_heartbeat();
      if (initial_hb_count_ < kInitialHbWatchCount) {
        ++initial_hb_count_;
      }
    }
  }

  // PDO TX 送信 (OPERATIONAL のみ)
  if (state_ == DeviceState::OPERATIONAL) {
    for (uint8_t ni = 0; ni < node_count_; ++ni) {
      NodeBase * node = nodes_[ni];
      // 同一 pdo_id を 1 回だけ送信するための dedup リスト
      uint16_t sent_pdo_ids[kMaxPdoPerNode];
      uint8_t  sent_count = 0;
      for (uint8_t pi = 0; pi < node->pdo_tx_count(); ++pi) {
        PdoTxEntry & e = node->pdo_tx_at(pi);
        bool periodic_due = (e.period_ms > 0) && ((now - e.last_tx_ms) >= e.period_ms);
        bool event_due = node->is_pdo_tx_requested(e.pdo_id);
        if (periodic_due || event_due) {
          // pdo_id が未送信なら fill して送信
          bool already_sent = false;
          for (uint8_t k = 0; k < sent_count; ++k) {
            if (sent_pdo_ids[k] == e.pdo_id) { already_sent = true; break; }
          }
          bool event_consumed = false;
          if (!already_sent) {
            uint8_t buf[64] = {};
            uint8_t len = node->fill_pdo_tx(e.pdo_id, buf, 64);
            if (len > 0) {
              CanFrame pdo_frame = make_standard_frame(e.pdo_id, buf, len);
              Status send_status = send_frame(pdo_frame);
              if (event_due && send_status == Status::OK) {
                event_consumed = true;
              }
            }
            if (sent_count < kMaxPdoPerNode) {
              sent_pdo_ids[sent_count++] = e.pdo_id;
            }
          } else if (event_due) {
            event_consumed = true;
          }
          if (periodic_due) {
            e.last_tx_ms = now;
          }
          if (event_consumed) {
            node->clear_pdo_tx_request(e.pdo_id);
          }
        }
      }
    }
  }
}

// ── フレーム受信ディスパッチ ──

void Device::handle_frame(const CanFrame & f)
{
  if (!is_management_frame(f)) {
    // Standard ID = PDO
    if (state_ == DeviceState::OPERATIONAL) {
      handle_pdo(f);
    }
    return;
  }

  ExtendedId eid = decode_extended_id(f.id);

  // NMT は常に処理 (衝突検出・状態制御のため)
  if (eid.function_code == FunctionCode::NMT) {
    handle_nmt(eid, f);
    return;
  }

  // ERROR / STOPPED 状態では NMT 以外を無視
  if (state_ == DeviceState::ERROR || state_ == DeviceState::STOPPED) {
    return;
  }

  // BULK FC は常に処理 (自身のTXセッション継続のため)
  if (eid.function_code == FunctionCode::BULK && eid.dst_dev == device_id_) {
    handle_bulk(eid, f);
    return;
  }

  switch (eid.function_code) {
    case FunctionCode::DISC:
      if (eid.dst_dev == device_id_ || eid.dst_dev == kBroadcastDeviceId) {
        handle_disc(eid, f);
      }
      break;
    case FunctionCode::PARAM:
      if (eid.dst_dev == device_id_) {
        handle_param(eid, f);
      }
      break;
    case FunctionCode::PDO_CFG:
      if (eid.dst_dev == device_id_ || eid.dst_dev == kBroadcastDeviceId) {
        handle_pdo_cfg(eid, f);
      }
      break;
    case FunctionCode::SERVICE:
      if (eid.dst_dev == device_id_) {
        handle_service(eid, f);
      }
      break;
    default:
      break;
  }
}

// ── NMT ──

void Device::handle_nmt(const ExtendedId & eid, const CanFrame & f)
{
  // ハートビート (context=0) の衝突検出
  if (eid.context == 0) {
    if (
      (listen_only_ || initial_hb_count_ < kInitialHbWatchCount) &&
      eid.src_dev == device_id_) {
      state_ = DeviceState::ERROR;
    }
    return;  // 他デバイスのハートビートは無視
  }

  // NMT コマンド: 自身または broadcast 宛のみ処理
  if (eid.dst_dev != device_id_ && eid.dst_dev != kBroadcastDeviceId) {
    return;
  }

  auto cmd = static_cast<NmtCommand>(eid.context);
  switch (cmd) {
    case NmtCommand::START:
      if (state_ == DeviceState::PREOP || state_ == DeviceState::STOPPED) {
        state_ = DeviceState::OPERATIONAL;
        if (!listen_only_) send_heartbeat();
      }
      break;
    case NmtCommand::STOP:
      state_ = DeviceState::STOPPED;
      break;
    case NmtCommand::ENTER_PREOP:
      if (state_ != DeviceState::ERROR) {
        state_ = DeviceState::PREOP;
        if (!listen_only_) send_heartbeat();
      }
      break;
    case NmtCommand::RESET_NODE:
      for (uint8_t i = 0; i < node_count_; ++i) {
        nodes_[i]->reset_pdos();
      }
      bulk_tx_.active = false;
      pdo_cfg_pending_.active = false;
      listen_only_ = true;
      listen_only_end_ms_ = get_time_ms_() + rand_ms_(kListenOnlyMinMs, kListenOnlyMaxMs);
      state_ = DeviceState::PREOP;
      break;
    default:
      break;
  }
  (void)f;
}

// ── ハートビート送信 ──

void Device::send_heartbeat()
{
  uint8_t payload[64] = {};
  uint32_t uptime = get_time_ms_() - start_ms_;

  // PREOP: ノード情報付き。最大 (64-6)/8 = 7 ノードまで
  uint8_t n = 0;
  if (state_ == DeviceState::PREOP) {
    static constexpr uint8_t kMaxNodesInHb =
      static_cast<uint8_t>((kCanFdMaxPayload - 6u) / 8u);
    n = (node_count_ < kMaxNodesInHb) ? node_count_ : kMaxNodesInHb;
  }

  payload[0] = static_cast<uint8_t>(state_);
  payload[1] = n;
  write_le32(payload + 2, uptime);

  for (uint8_t i = 0; i < n; ++i) {
    uint8_t * base = payload + 6 + i * 8;
    write_le32(base, nodes_[i]->schema_hash());
    base[4] = nodes_[i]->local_id();
    // base[5..7] = reserved (0)
  }

  ExtendedId eid;
  eid.function_code = FunctionCode::NMT;
  eid.src_dev = device_id_;
  eid.src_node = 0;
  eid.dst_dev = kBroadcastDeviceId;
  eid.dst_node = kBroadcastNodeId;
  eid.context = 0;

  uint8_t len = static_cast<uint8_t>(6u + n * 8u);
  CanFrame frame = make_extended_frame(eid, payload, len);
  send_frame(frame);

  last_heartbeat_ms_ = get_time_ms_();
}

// ── DISC ──

void Device::handle_disc(const ExtendedId & eid, const CanFrame & f)
{
  NodeBase * node = find_node(eid.dst_node);
  if (!node) return;

  // BULK TXが既に進行中の場合は受け付けない
  if (bulk_tx_.active) return;

  start_bulk_tx(
    eid.src_dev, eid.src_node, 0, BulkPayloadType::DESCRIPTOR, node->descriptor_blob(),
    static_cast<uint32_t>(node->descriptor_blob_size()));
  (void)f;
}

// ── BULK TX ──

void Device::start_bulk_tx(
  uint8_t dst_dev, uint8_t dst_node, uint8_t channel, BulkPayloadType payload_type,
  const uint8_t * blob, uint32_t len)
{
  bulk_tx_.active = true;
  bulk_tx_.dst_dev = dst_dev;
  bulk_tx_.dst_node = dst_node;
  bulk_tx_.channel = channel;
  bulk_tx_.payload_type = payload_type;
  bulk_tx_.data = blob;
  bulk_tx_.total = len;
  bulk_tx_.sent = 0;
  bulk_tx_.sequence = 1;
  bulk_tx_.waiting_fc = false;

  // First Frame 送信
  // [0] type=0, [1] payload_type, [2:5] total_len (LE u32), [6..63] data
  uint8_t frame_payload[64] = {};
  frame_payload[0] = static_cast<uint8_t>(BulkFrameType::FIRST_FRAME);
  frame_payload[1] = static_cast<uint8_t>(payload_type);
  write_le32(frame_payload + 2, len);

  uint32_t first_data = (len < 58u) ? len : 58u;
  if (first_data > 0) {
    std::memcpy(frame_payload + 6, blob, first_data);
  }
  bulk_tx_.sent = first_data;

  ExtendedId tx_eid;
  tx_eid.function_code = FunctionCode::BULK;
  tx_eid.src_dev = device_id_;
  tx_eid.src_node = 0;
  tx_eid.dst_dev = dst_dev;
  tx_eid.dst_node = dst_node;
  tx_eid.context = channel;

  CanFrame frame =
    make_extended_frame(tx_eid, frame_payload, static_cast<uint8_t>(6u + first_data));
  send_frame(frame);

  if (bulk_tx_.sent >= bulk_tx_.total) {
    bulk_tx_.active = false;
  } else {
    bulk_tx_.waiting_fc = true;
  }
}

void Device::continue_bulk_tx()
{
  if (!bulk_tx_.active || bulk_tx_.waiting_fc) return;

  ExtendedId tx_eid;
  tx_eid.function_code = FunctionCode::BULK;
  tx_eid.src_dev = device_id_;
  tx_eid.src_node = 0;
  tx_eid.dst_dev = bulk_tx_.dst_dev;
  tx_eid.dst_node = bulk_tx_.dst_node;
  tx_eid.context = bulk_tx_.channel;

  while (bulk_tx_.sent < bulk_tx_.total) {
    uint8_t frame_payload[64] = {};
    frame_payload[0] = static_cast<uint8_t>(BulkFrameType::CONSECUTIVE_FRAME);
    frame_payload[1] = bulk_tx_.sequence;
    bulk_tx_.sequence = static_cast<uint8_t>((bulk_tx_.sequence + 1u) & 0xFFu);

    uint32_t remaining = bulk_tx_.total - bulk_tx_.sent;
    uint32_t chunk = (remaining < 62u) ? remaining : 62u;
    std::memcpy(frame_payload + 2, bulk_tx_.data + bulk_tx_.sent, chunk);
    bulk_tx_.sent += chunk;

    CanFrame frame =
      make_extended_frame(tx_eid, frame_payload, static_cast<uint8_t>(2u + chunk));
    send_frame(frame);
  }

  bulk_tx_.active = false;
}

// ── BULK RX (FC受信のみ) ──

void Device::handle_bulk(const ExtendedId & eid, const CanFrame & f)
{
  if (f.dlc < 1) return;

  auto frame_type = static_cast<BulkFrameType>(f.data[0]);

  switch (frame_type) {
    case BulkFrameType::FLOW_CONTROL: {
      if (!bulk_tx_.active || !bulk_tx_.waiting_fc) return;
      if (f.dlc < 2) return;

      auto fc_flag = static_cast<BulkFcFlag>(f.data[1]);
      if (fc_flag == BulkFcFlag::ABORT) {
        bulk_tx_.active = false;
        bulk_tx_.waiting_fc = false;
      } else if (fc_flag == BulkFcFlag::CONTINUE) {
        bulk_tx_.waiting_fc = false;
        continue_bulk_tx();
      }
      // WAIT: 何もしない
      break;
    }
    case BulkFrameType::FIRST_FRAME:
    case BulkFrameType::CONSECUTIVE_FRAME:
      // デバイス側での大きなサービスリクエスト受信は未対応
      break;
  }
  (void)eid;
}

// ── PARAM ──

void Device::handle_param(const ExtendedId & eid, const CanFrame & f)
{
  auto cmd = static_cast<ParamCommand>(eid.context);

  NodeBase * node = find_node(eid.dst_node);
  if (!node) return;

  if (cmd == ParamCommand::READ) {
    if (f.dlc < 1) return;
    uint8_t param_idx = f.data[0];

    uint8_t out[62] = {};
    uint8_t out_size = 0;
    Status s = node->on_param_read(param_idx, out, out_size);

    ExtendedId res_eid;
    res_eid.function_code = FunctionCode::PARAM;
    res_eid.src_dev = device_id_;
    res_eid.src_node = node->local_id();
    res_eid.dst_dev = eid.src_dev;
    res_eid.dst_node = eid.src_node;
    res_eid.context = static_cast<uint8_t>(ParamCommand::READ_RES);

    uint8_t payload[64] = {};
    payload[0] = param_idx;
    payload[1] = static_cast<uint8_t>(s == Status::OK ? ParamStatus::OK : ParamStatus::ERROR);
    if (s == Status::OK && out_size > 0) {
      std::memcpy(payload + 2, out, out_size);
    }
    CanFrame res_frame =
      make_extended_frame(res_eid, payload, static_cast<uint8_t>(2u + out_size));
    send_frame(res_frame);

  } else if (cmd == ParamCommand::WRITE) {
    if (f.dlc < 1) return;
    uint8_t param_idx = f.data[0];
    const uint8_t * data = f.data.data() + 1;
    uint8_t data_size = (f.dlc > 1) ? static_cast<uint8_t>(f.dlc - 1) : 0;

    Status s = node->on_param_write(param_idx, data, data_size);

    ExtendedId res_eid;
    res_eid.function_code = FunctionCode::PARAM;
    res_eid.src_dev = device_id_;
    res_eid.src_node = node->local_id();
    res_eid.dst_dev = eid.src_dev;
    res_eid.dst_node = eid.src_node;
    res_eid.context = static_cast<uint8_t>(ParamCommand::WRITE_RES);

    uint8_t payload[2] = {param_idx, static_cast<uint8_t>(
                                       s == Status::OK ? ParamStatus::OK : ParamStatus::ERROR)};
    CanFrame res_frame = make_extended_frame(res_eid, payload, 2);
    send_frame(res_frame);
  }
}

// ── PDO_CFG ──

void Device::handle_pdo_cfg(const ExtendedId & eid, const CanFrame & f)
{
  uint8_t ctx = eid.context;

  if (ctx == static_cast<uint8_t>(PdoCfgSequence::BEGIN)) {
    if (f.dlc < 7) return;
    pdo_cfg_pending_.active = true;
    pdo_cfg_pending_.pdo_id = read_le16(f.data.data());
    pdo_cfg_pending_.dir = static_cast<PdoCfgDirection>(f.data[2]);
    pdo_cfg_pending_.num_entries = f.data[3];
    pdo_cfg_pending_.period_ms = read_le16(f.data.data() + 4);
    pdo_cfg_pending_.total_size = f.data[6];
    pdo_cfg_pending_.received = 0;

  } else if (ctx >= 1u && ctx <= 0x1Eu) {
    // ENTRY
    if (!pdo_cfg_pending_.active) return;
    if (f.dlc < 5) return;
    if (pdo_cfg_pending_.received >= 30) return;

    PdoCfgEntry & entry = pdo_cfg_pending_.entries[pdo_cfg_pending_.received++];
    entry.local_node_id = f.data[0];
    entry.topic_index = f.data[1];
    entry.field_index = f.data[2];
    entry.offset = f.data[3];
    entry.size = f.data[4];

  } else if (ctx == static_cast<uint8_t>(PdoCfgSequence::COMMIT)) {
    if (f.dlc < 3) return;
    uint16_t pdo_id = read_le16(f.data.data());
    auto action = static_cast<PdoCfgAction>(f.data[2]);

    PdoCfgStatus status = PdoCfgStatus::OK;

    if (action == PdoCfgAction::APPLY && pdo_cfg_pending_.active &&
        pdo_cfg_pending_.pdo_id == pdo_id) {
      status = commit_pdo_cfg();
    } else if (action == PdoCfgAction::DELETE) {
      for (uint8_t ni = 0; ni < node_count_; ++ni) {
        nodes_[ni]->clear_pdo(pdo_id);
      }
    }

    pdo_cfg_pending_.active = false;

    // PDO_CFG_ACK 送信
    ExtendedId res_eid;
    res_eid.function_code = FunctionCode::PDO_CFG;
    res_eid.src_dev = device_id_;
    res_eid.src_node = 0;
    res_eid.dst_dev = eid.src_dev;
    res_eid.dst_node = eid.src_node;
    res_eid.context = 0;

    uint8_t payload[3] = {};
    write_le16(payload, pdo_id);
    payload[2] = static_cast<uint8_t>(status);
    CanFrame ack_frame = make_extended_frame(res_eid, payload, 3);
    send_frame(ack_frame);
  }
}

PdoCfgStatus Device::commit_pdo_cfg()
{
  PdoCfgStatus overall = PdoCfgStatus::OK;

  for (uint8_t i = 0; i < pdo_cfg_pending_.received; ++i) {
    const PdoCfgEntry & entry = pdo_cfg_pending_.entries[i];

    NodeBase * node = find_node(entry.local_node_id);
    if (!node) continue;

    if (pdo_cfg_pending_.dir == PdoCfgDirection::TX) {
      // 同一 (pdo_id, topic_index, field_index) が既に存在すればスキップ
      bool found = false;
      for (uint8_t pi = 0; pi < node->pdo_tx_count(); ++pi) {
        const PdoTxEntry & existing = node->pdo_tx_at(pi);
        if (existing.pdo_id == pdo_cfg_pending_.pdo_id &&
            existing.topic_index == entry.topic_index &&
            existing.field_index == entry.field_index) {
          found = true;
          break;
        }
      }
      if (!found) {
        PdoTxEntry tx_entry;
        tx_entry.pdo_id      = pdo_cfg_pending_.pdo_id;
        tx_entry.topic_index = entry.topic_index;
        tx_entry.field_index = entry.field_index;
        tx_entry.offset      = entry.offset;
        tx_entry.size        = entry.size;
        tx_entry.period_ms   = pdo_cfg_pending_.period_ms;
        tx_entry.last_tx_ms  = 0;
        Status s = node->add_pdo_tx(tx_entry);
        if (s != Status::OK) overall = PdoCfgStatus::NO_RESOURCE;
      }
    } else {
      // RX
      PdoRxEntry rx_entry;
      rx_entry.pdo_id      = pdo_cfg_pending_.pdo_id;
      rx_entry.topic_index = entry.topic_index;
      rx_entry.field_index = entry.field_index;
      rx_entry.offset      = entry.offset;
      rx_entry.size        = entry.size;
      Status s = node->add_pdo_rx(rx_entry);
      if (s != Status::OK) overall = PdoCfgStatus::NO_RESOURCE;
    }
  }

  return overall;
}

// ── SERVICE ──

void Device::handle_service(const ExtendedId & eid, const CanFrame & f)
{
  if (f.dlc < 1) return;

  NodeBase * node = find_node(eid.dst_node);
  if (!node) return;

  uint8_t svc_idx = f.data[0];
  const uint8_t * req_data = f.data.data() + 1;
  uint8_t req_size = (f.dlc > 1) ? static_cast<uint8_t>(f.dlc - 1) : 0;

  uint8_t res_buf[62] = {};
  uint8_t res_size = 0;
  Status s = node->on_service_req(svc_idx, req_data, req_size, res_buf, res_size);

  ExtendedId res_eid;
  res_eid.function_code = FunctionCode::SERVICE;
  res_eid.src_dev = device_id_;
  res_eid.src_node = node->local_id();
  res_eid.dst_dev = eid.src_dev;
  res_eid.dst_node = eid.src_node;
  res_eid.context = eid.context;  // 同一 seq_id

  if (res_size <= 62u) {
    uint8_t payload[64] = {};
    payload[0] = svc_idx;
    payload[1] =
      static_cast<uint8_t>(s == Status::OK ? ServiceStatus::OK : ServiceStatus::ERROR);
    if (res_size > 0) {
      std::memcpy(payload + 2, res_buf, res_size);
    }
    CanFrame res_frame =
      make_extended_frame(res_eid, payload, static_cast<uint8_t>(2u + res_size));
    send_frame(res_frame);
  } else {
    // TODO: BULK SERVICE_RES 対応 (res_size > 62 の場合)
    // 現在は NO_RESOURCE エラーで応答
    uint8_t payload[2] = {svc_idx, static_cast<uint8_t>(ServiceStatus::ERROR)};
    CanFrame res_frame = make_extended_frame(res_eid, payload, 2);
    send_frame(res_frame);
  }
}

// ── PDO 受信 ──

void Device::handle_pdo(const CanFrame & f)
{
  uint16_t pdo_id = static_cast<uint16_t>(f.id & 0x7FFu);

  for (uint8_t ni = 0; ni < node_count_; ++ni) {
    NodeBase * node = nodes_[ni];
    for (uint8_t pi = 0; pi < node->pdo_rx_count(); ++pi) {
      if (node->pdo_rx_at(pi).pdo_id == pdo_id) {
        node->on_pdo_rx(pdo_id, f.data.data(), f.dlc);
        break;  // 1ノードにつき1回
      }
    }
  }
}

// ── EMCY 送信 ──

Status Device::send_emcy(
  uint8_t local_node_id, uint16_t error_register, const uint8_t * error_data,
  uint8_t error_data_size)
{
  ExtendedId eid;
  eid.function_code = FunctionCode::EMCY;
  eid.src_dev = device_id_;
  eid.src_node = local_node_id;
  eid.dst_dev = kBroadcastDeviceId;
  eid.dst_node = kBroadcastNodeId;
  eid.context = 0;

  uint8_t payload[64] = {};
  write_le16(payload, error_register);

  uint8_t data_size = (error_data_size > 62u) ? 62u : error_data_size;
  if (error_data && data_size > 0) {
    std::memcpy(payload + 2, error_data, data_size);
  }

  CanFrame frame =
    make_extended_frame(eid, payload, static_cast<uint8_t>(2u + data_size));
  return send_frame(frame);
}

// ── ヘルパー ──

NodeBase * Device::find_node(uint8_t local_node_id)
{
  for (uint8_t i = 0; i < node_count_; ++i) {
    if (nodes_[i]->local_id() == local_node_id) {
      return nodes_[i];
    }
  }
  return nullptr;
}

Status Device::send_frame(const CanFrame & f) { return can_->send(f); }

Status Device::send_pdo_trampoline(
  uint16_t pdo_id, const uint8_t * data, uint8_t len, void * ctx)
{
  auto *   self  = static_cast<Device *>(ctx);
  CanFrame frame = make_standard_frame(pdo_id, data, len);
  return self->send_frame(frame);
}

}  // namespace protocan::device
