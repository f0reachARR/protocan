#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <optional>
#include <functional>

namespace protocan {

// ════════════════════════════════════════════════════════════════
// CAN ID 定数
// ════════════════════════════════════════════════════════════════

/// マスター（ブリッジ）の Device ID (spec §4.1)
static constexpr uint8_t kMasterDeviceId = 0;

/// デバイス用 Device ID の範囲 (spec §4.1)
static constexpr uint8_t kMinDeviceId = 1;
static constexpr uint8_t kMaxDeviceId = 14;

/// ブロードキャスト Device ID (spec §4.1)
static constexpr uint8_t kBroadcastDeviceId = 0x0F;

/// ブロードキャスト Local Node ID (spec §4.1)
static constexpr uint8_t kBroadcastNodeId = 0x3F;

/// Local Node ID の最大値 (spec §4.1)
static constexpr uint8_t kMaxLocalNodeId = 62;

/// CAN FD 最大ペイロード長
static constexpr size_t kCanFdMaxPayload = 64;

// ════════════════════════════════════════════════════════════════
// PDO 優先度帯域 (spec §4.3)
// ════════════════════════════════════════════════════════════════

static constexpr uint16_t kPdoHighPriorityMin = 0x001;
static constexpr uint16_t kPdoHighPriorityMax = 0x0FF;
static constexpr uint16_t kPdoMidPriorityMin  = 0x100;
static constexpr uint16_t kPdoMidPriorityMax  = 0x3FF;
static constexpr uint16_t kPdoLowPriorityMin  = 0x400;
static constexpr uint16_t kPdoLowPriorityMax  = 0x7FF;

// ════════════════════════════════════════════════════════════════
// Function Code (spec §4.2)
// ════════════════════════════════════════════════════════════════

enum class FunctionCode : uint8_t {
    NMT      = 0x0,
    EMCY     = 0x1,
    DISC     = 0x2,
    PARAM    = 0x3,
    PDO_CFG  = 0x4,
    BULK     = 0x5,
    SYNC     = 0x6,
    // 0x7 reserved
    SERVICE  = 0x8,
};

// ════════════════════════════════════════════════════════════════
// NMT (spec §5.1)
// ════════════════════════════════════════════════════════════════

/// デバイス状態
enum class DeviceState : uint8_t {
    PREOP       = 0,
    OPERATIONAL = 1,
    ERROR       = 2,
    STOPPED     = 3,
};

/// NMT 制御コマンド (ctx フィールド)
enum class NmtCommand : uint8_t {
    START       = 0x01,
    STOP        = 0x02,
    ENTER_PREOP = 0x03,
    RESET_NODE  = 0x04,
};

/// Heartbeat 内の個別ノード情報
struct HeartbeatNodeEntry {
    uint32_t schema_hash;
    uint8_t  local_node_id;
};

/// Heartbeat 構造体
struct Heartbeat {
    DeviceState state;
    uint8_t     num_nodes;
    uint32_t    uptime_ms;
    std::vector<HeartbeatNodeEntry> nodes;
};

// ════════════════════════════════════════════════════════════════
// EMCY (spec §5.2)
// ════════════════════════════════════════════════════════════════

/// EMCY error_register ビット定義
enum class EmcyCategory : uint16_t {
    GENERIC_ERROR  = (1 << 0),
    CURRENT        = (1 << 1),
    VOLTAGE        = (1 << 2),
    TEMPERATURE    = (1 << 3),
    COMMUNICATION  = (1 << 4),
    DEVICE_PROFILE = (1 << 5),
    HARDWARE       = (1 << 6),
};

struct EmcyMessage {
    uint16_t             error_register;
    std::vector<uint8_t> error_data;  // Node Type 依存の詳細情報
};

// ════════════════════════════════════════════════════════════════
// PARAM (spec §5.4)
// ════════════════════════════════════════════════════════════════

enum class ParamCommand : uint8_t {
    READ      = 0x01,
    WRITE     = 0x02,
    READ_RES  = 0x81,
    WRITE_RES = 0x82,
};

enum class ParamStatus : uint8_t {
    OK        = 0,
    NOT_FOUND = 1,
    ERROR     = 2,
};

// ════════════════════════════════════════════════════════════════
// PDO_CFG (spec §5.5)
// ════════════════════════════════════════════════════════════════

enum class PdoCfgSequence : uint8_t {
    BEGIN  = 0x00,
    // ENTRY = 0x01..0x1E
    COMMIT = 0x1F,
};

enum class PdoCfgDirection : uint8_t {
    TX = 0,  // TX from device
    RX = 1,  // RX to device
};

enum class PdoCfgAction : uint8_t {
    APPLY  = 0,
    DELETE = 1,
};

enum class PdoCfgStatus : uint8_t {
    OK          = 0,
    INVALID     = 1,
    NO_RESOURCE = 2,
};

/// PDO_CFG BEGIN ペイロード
struct PdoCfgBegin {
    uint16_t pdo_id;
    PdoCfgDirection direction;
    uint8_t  num_entries;
    uint16_t period_ms;
    uint8_t  total_size;
};

/// PDO_CFG ENTRY ペイロード
struct PdoCfgEntry {
    uint8_t local_node_id;
    uint8_t topic_index;
    uint8_t field_index;
    uint8_t offset;
    uint8_t size;
};

/// PDO_CFG COMMIT ペイロード
struct PdoCfgCommit {
    uint16_t     pdo_id;
    PdoCfgAction action;
};

/// PDO_CFG ACK ペイロード
struct PdoCfgAck {
    uint16_t     pdo_id;
    PdoCfgStatus status;
};

// ════════════════════════════════════════════════════════════════
// BULK (spec §5.6)
// ════════════════════════════════════════════════════════════════

enum class BulkFrameType : uint8_t {
    FIRST_FRAME       = 0,
    CONSECUTIVE_FRAME = 1,
    FLOW_CONTROL      = 2,
};

enum class BulkPayloadType : uint8_t {
    DESCRIPTOR  = 0,
    SERVICE_REQ = 1,
    SERVICE_RES = 2,
};

enum class BulkFcFlag : uint8_t {
    CONTINUE = 0,
    WAIT     = 1,
    ABORT    = 2,
};

/// BULK チャンネルの最大数 (0–30, 31 は予約)
static constexpr uint8_t kBulkMaxChannel = 30;
static constexpr uint8_t kBulkReservedChannel = 31;

// ════════════════════════════════════════════════════════════════
// SERVICE (spec §5.8)
// ════════════════════════════════════════════════════════════════

enum class ServiceStatus : uint8_t {
    OK        = 0,
    ERROR     = 1,
    NOT_FOUND = 2,
};

/// サービスリクエストタイムアウト (推奨値, spec §5.8)
static constexpr uint32_t kServiceTimeoutMs = 1000;

// ════════════════════════════════════════════════════════════════
// 共通ステータスコード
// ════════════════════════════════════════════════════════════════

enum class Status {
    OK,
    ERROR,
    TIMEOUT,
    INVALID_ARGUMENT,
    NO_RESOURCE,
    NOT_FOUND,
    BUSY,
};

// ════════════════════════════════════════════════════════════════
// PDO 優先度 (spec §2.2 method option)
// ════════════════════════════════════════════════════════════════

enum class PdoPriority : uint8_t {
    HIGH = 0,  // 0-2 → High band
    MID  = 3,  // 3-4 → Mid band
    LOW  = 5,  // 5-7 → Low band
};

/// priority 値 (0-7) から PDO 優先度帯域へ変換
inline PdoPriority priority_to_band(uint32_t priority) {
    if (priority <= 2) return PdoPriority::HIGH;
    if (priority <= 4) return PdoPriority::MID;
    return PdoPriority::LOW;
}

}  // namespace protocan
