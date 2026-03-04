# ProtoCAN — CAN FD プラグ・アンド・プレイ通信システム仕様 (Draft v0.3)

## 1. 概要

ProtoCAN は、CAN FD バス上で複数の組み込みデバイスと PC（ROS 2）が双方向通信を行うためのプロトコルである。

### v0.2 からの主な変更

- **スキーマハッシュ**: `node_type_id` + `schema_version` を廃止し、
  スキーマ構造から計算される 32-bit ハッシュで一致判定
- **ROS 2 既存メッセージ型の再利用**: `ros2_msg_type` / `ros2_field` オプションで
  `sensor_msgs/Imu` 等の標準型に直接マッピング可能

### 用語定義

| 用語 | 定義 |
|------|------|
| **Device** | 物理的な 1 デバイス（1 MCU / 1 CAN トランシーバ） |
| **Node** | Device 上の論理的な機能単位。1 `.proto` = 1 Node Type |
| **Node Instance** | 同一 Node Type の複数インスタンス（例: 左 / 右モーター） |
| **Topic** | Node が送受信するデータストリーム（ROS 2 トピックに対応） |
| **Param** | Node のランタイム設定値（ROS 2 パラメータに対応） |
| **PDO** | Process Data Object — マスター割当の Standard CAN ID + フレームレイアウト |
| **Schema Hash** | `.proto` の構造的内容から計算される 32-bit 識別子 |
| **Master** | PC 側の ProtoCAN ブリッジ |

### アーキテクチャ全体像

```
┌───────────────────────────────────────────────────────────────┐
│  .proto スキーマ群 (Node Type 定義)                              │
│  motor.proto, imu.proto, gpio.proto, ...                      │
└──────────┬────────────────────────────┬───────────────────────┘
           │  protoc-gen-protocan        │  protoc-gen-protocan
           ▼                            ▼
┌────────────────────────┐   ┌────────────────────────────────┐
│  ファームウェア用 C Lib    │   │  ROS 2 ブリッジ                  │
│  - packed エンコーダ      │   │  - ハッシュ照合 → 即セットアップ    │
│  - ディスクリプタ (ROM)    │   │  - PDO マッピング & ルーティング   │
│  - schema_hash (ROM)    │   │  - 既存 ROS 2 msg 型で publish   │
│  - PDO テーブル (RAM)     │   │  - トピック/パラメータ自動生成      │
└─────────┬──────────────┘   └───────────────┬────────────────┘
          │                                   │
          ▼                                   ▼
┌───────────────────────────────────────────────────────────────┐
│                        CAN FD バス                             │
│  [Standard ID: PDO データ]    [Extended ID: 管理・ディスカバリ]    │
│                                                               │
│  Device A          Device B           PC (Master)             │
│  ├─ Node: Motor×2  ├─ Node: IMU       ProtoCAN Bridge         │
│  └─ Node: GPIO     └─ Node: GPIO                              │
└───────────────────────────────────────────────────────────────┘
```

---

## 2. スキーマハッシュ

### 2.1 目的

- デバイスとマスターが **同じ `.proto` ファイルから生成されたか** を高速判定
- 明示的な ID / バージョン番号の手動管理を不要にする
- ハートビート（数バイト）だけでスキーマの一致/不一致を判定し、
  一致すればディスクリプタ転送をスキップ

### 2.2 ハッシュ入力（正規化表現）

スキーマの **構造的内容** のみからハッシュを計算する。
コメント、空白、`ros2_*` オプション等の「CAN バス上の互換性に影響しない情報」は入力に含めない。

```
正規化手順:
1. ファイル内の全 message を protocan.topic / protocan.param オプション付きのものだけ抽出
2. message を topic_index / param_index 順（= ファイル内出現順）にソート
3. 各 message について以下をバイト列として連結:
   a. message 種別: 'T' (topic) or 'P' (param)
   b. TopicOptions: direction (1B), periodic (1B), priority (1B)
      または ParamOptions: read_only (1B)
   c. num_fields (1B)
   d. 各 field を field_number 順に:
      - field_number (varint)
      - field_type enum (1B)
4. 全 message のバイト列を連結
5. FNV-1a 32-bit ハッシュを計算
```

### 2.3 ハッシュの性質

- **同一構造 → 同一ハッシュ**: フィールド名・ROS 2 マッピングの変更はハッシュに影響しない
- **構造変更 → ハッシュ変化**: フィールド追加/削除/型変更/並べ替えで変化する
- **衝突**: 32-bit なので理論上 2^16 種類程度で衝突確率が無視できなくなるが、
  実運用では十分。衝突時はディスクリプタの full 比較にフォールバック

### 2.4 スキーマ互換性の判定フロー

```
デバイスのハートビートに schema_hash が含まれる
                    │
                    ▼
        マスターの既知スキーマに同じハッシュがある？
              │                    │
             YES                   NO
              │                    │
              ▼                    ▼
    コンパイル済みスキーマで      ディスクリプタを
    即座にセットアップ            バルク転送で取得
    (ディスクリプタ転送不要)       → 動的デコーダ構築
```

---

## 3. スキーマ定義 (.proto)

### 3.1 カスタムオプション定義

```protobuf
// protocan/options.proto
syntax = "proto3";
package protocan;

import "google/protobuf/descriptor.proto";

// ══════════════════════════════════════════════════
//  ファイルレベル = Node Type 定義
// ══════════════════════════════════════════════════

message NodeTypeOptions {
  string node_type_name   = 1;  // 人間可読名 (例: "BLDCMotor")
  string ros2_namespace   = 2;  // ROS 2 名前空間プレフィックス（省略時は自動）
  // ※ node_type_id, schema_version は廃止 → schema_hash に統合
}

extend google.protobuf.FileOptions {
  NodeTypeOptions node_type = 50000;
}

// ══════════════════════════════════════════════════
//  メッセージレベル = Topic / Param
// ══════════════════════════════════════════════════

enum Direction {
  DEVICE_TO_BUS = 0;  // デバイスが送信元
  BUS_TO_DEVICE = 1;  // デバイスが受信側
}

message TopicOptions {
  Direction direction   = 1;
  bool     periodic     = 2;  // true: 周期送信（周期値はランタイムパラメータ）
  uint32   priority     = 3;  // 0=最高, 7=最低 — PDO 割当ヒント
  string   ros2_topic   = 4;  // ROS 2 トピック名オーバーライド
  string   ros2_msg_type = 5; // 既存 ROS 2 メッセージ型 (例: "sensor_msgs/msg/Imu")
                               // 省略時: カスタム .msg を自動生成
}

message ParamOptions {
  bool   read_only     = 1;
  string ros2_param    = 2;
  string ros2_msg_type = 3;  // パラメータ用の ROS 2 型オーバーライド
}

extend google.protobuf.MessageOptions {
  TopicOptions topic = 50001;
  ParamOptions param = 50002;
}

// ══════════════════════════════════════════════════
//  フィールドレベル = ROS 2 フィールドマッピング
// ══════════════════════════════════════════════════

message FieldOptions {
  string ros2_field = 1;  // ROS 2 メッセージ内のフィールドパス
                           // (例: "linear_acceleration.x")
                           // 省略時: proto フィールド名をそのまま使用
}

extend google.protobuf.FieldOptions {
  FieldOptions field = 50003;
}
```

### 3.2 Node Type 定義例: IMU (ROS 2 標準型を再利用)

```protobuf
// imu.proto
syntax = "proto3";
package imu;

import "protocan/options.proto";

option (protocan.node_type) = {
  node_type_name: "IMU",
  ros2_namespace: "imu"
};

// sensor_msgs/msg/Imu にマッピング
// CAN 上は 24 bytes の flat struct、ROS 2 側は標準 Imu メッセージ
message ImuData {
  option (protocan.topic) = {
    direction: DEVICE_TO_BUS,
    periodic: true,
    priority: 1,
    ros2_msg_type: "sensor_msgs/msg/Imu"
  };

  // Proto field → ROS 2 nested field のマッピング
  // sensor_msgs/Imu.linear_acceleration.x ← accel_x
  float accel_x = 1 [(protocan.field) = { ros2_field: "linear_acceleration.x" }];
  float accel_y = 2 [(protocan.field) = { ros2_field: "linear_acceleration.y" }];
  float accel_z = 3 [(protocan.field) = { ros2_field: "linear_acceleration.z" }];
  float gyro_x  = 4 [(protocan.field) = { ros2_field: "angular_velocity.x" }];
  float gyro_y  = 5 [(protocan.field) = { ros2_field: "angular_velocity.y" }];
  float gyro_z  = 6 [(protocan.field) = { ros2_field: "angular_velocity.z" }];

  // ※ sensor_msgs/Imu の orientation, covariance 等は
  //   CAN 上で送信しないフィールド → ROS 2 側でゼロ埋め or 定数設定
}
```

**生成される ROS 2 コード（概念）**:

```cpp
// 自動生成: imu_converter.cpp
#include <sensor_msgs/msg/imu.hpp>

sensor_msgs::msg::Imu convert_imu_data(const uint8_t* can_payload) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;  // ブリッジ設定から

    // packed layout → ROS 2 nested fields
    msg.linear_acceleration.x = read_float_le(can_payload, 0);
    msg.linear_acceleration.y = read_float_le(can_payload, 4);
    msg.linear_acceleration.z = read_float_le(can_payload, 8);
    msg.angular_velocity.x    = read_float_le(can_payload, 12);
    msg.angular_velocity.y    = read_float_le(can_payload, 16);
    msg.angular_velocity.z    = read_float_le(can_payload, 20);

    // マッピングされていないフィールドはデフォルト値
    // orientation: identity quaternion
    msg.orientation.w = 1.0;
    return msg;
}
```

### 3.3 Node Type 定義例: BLDC モータードライバ (カスタム型)

```protobuf
// bldc_motor.proto
syntax = "proto3";
package bldc_motor;

import "protocan/options.proto";

option (protocan.node_type) = {
  node_type_name: "BLDCMotor",
  ros2_namespace: "motor"
};

// ──── Topics ────

// ros2_msg_type 省略 → カスタム .msg を自動生成
message MotorStatus {
  option (protocan.topic) = {
    direction: DEVICE_TO_BUS,
    periodic: true,
    priority: 3
  };
  float  current_a     = 1;  // 4B
  float  velocity_rps  = 2;  // 4B
  float  temperature_c = 3;  // 4B
  uint32 error_flags   = 4;  // 4B  → 合計 16 bytes
}

message EncoderFeedback {
  option (protocan.topic) = {
    direction: DEVICE_TO_BUS,
    periodic: true,
    priority: 1
  };
  int32 position_cnt = 1;
  int32 velocity_cps = 2;    // → 合計 8 bytes
}

// geometry_msgs/msg/Twist にマッピングする例
// 6DoF コマンド (線速度 + 角速度)
message MotorCommand {
  option (protocan.topic) = {
    direction: BUS_TO_DEVICE,
    periodic: false,
    priority: 2,
    ros2_msg_type: "geometry_msgs/msg/Twist"
  };
  float linear_x  = 1 [(protocan.field) = { ros2_field: "linear.x" }];
  float linear_y  = 2 [(protocan.field) = { ros2_field: "linear.y" }];
  float linear_z  = 3 [(protocan.field) = { ros2_field: "linear.z" }];
  float angular_x = 4 [(protocan.field) = { ros2_field: "angular.x" }];
  float angular_y = 5 [(protocan.field) = { ros2_field: "angular.y" }];
  float angular_z = 6 [(protocan.field) = { ros2_field: "angular.z" }];
}

// ──── Params ────

message PidGains {
  option (protocan.param) = { read_only: false };
  float kp = 1;
  float ki = 2;
  float kd = 3;
}

message DeviceInfo {
  option (protocan.param) = { read_only: true };
  uint32 firmware_version = 1;
  uint32 serial_number    = 2;
}
```

### 3.4 ros2_msg_type の動作まとめ

| `ros2_msg_type` | `ros2_field` | 動作 |
|-----------------|-------------|------|
| 省略 | 省略 | カスタム `.msg` を自動生成。フィールド名はそのまま |
| 指定 | 省略 | 既存 ROS 2 型を使用。proto フィールド名で直接マッピング |
| 指定 | 指定 | 既存 ROS 2 型を使用。ネスト構造へのパスマッピング |
| 省略 | 指定 | エラー（ros2_field は ros2_msg_type と併用が前提） |

**マッピングされないフィールドの扱い**:

ROS 2 メッセージ側にあるが proto 側にマッピングがないフィールドは:

- 数値型: `0` で初期化
- `Header.stamp`: ブリッジが受信時刻を設定
- `Header.frame_id`: ブリッジ設定の YAML から取得
- 配列型（covariance 等）: ゼロ埋め

これらのデフォルト動作はブリッジの YAML 設定で上書き可能:

```yaml
# protocan_bridge 設定
unmapped_field_defaults:
  "sensor_msgs/msg/Imu":
    "orientation.w": 1.0
    "orientation_covariance[0]": -1.0  # unknown を示す慣習
  "geometry_msgs/msg/Twist":
    # (全フィールドがマッピング済みなので不要)
```

### 3.5 マルチノードデバイスの構成

```toml
# device_manifest.toml
[device]
device_type_name = "DualMotorBoard"

[[nodes]]
proto = "bldc_motor.proto"
instance_count = 2
instance_names = ["left", "right"]

[[nodes]]
proto = "imu.proto"
instance_count = 1

[[nodes]]
proto = "gpio.proto"
instance_count = 1
```

protoc プラグインの処理:

1. 各 `.proto` の schema_hash を計算
2. 各 Node Instance に local_node_id を自動割当
3. デバイスディスクリプタを生成（各ノードの schema_hash を含む）
4. ファームウェア用 C コード / ROS 2 コンバータを生成

### 3.6 型マッピング（Proto型 → バイナリ表現）

| Proto 型 | CAN バイナリ | サイズ |
|----------|-------------|-------|
| `bool`   | `uint8` (0/1) | 1 byte |
| `uint32` | LE u32 | 4 bytes |
| `int32`  | LE i32 | 4 bytes |
| `float`  | IEEE 754 LE | 4 bytes |
| `double` | IEEE 754 LE | 8 bytes |
| `uint64` | LE u64 | 8 bytes |
| `int64`  | LE i64 | 8 bytes |

> 可変長 (`string`, `bytes`, `repeated`) / `oneof` は v0.3 では非サポート。

---

## 4. CAN フレーム設計

### 4.1 二層構成

| 層 | CAN ID 形式 | 用途 |
|----|------------|------|
| **データ層** | Standard ID (11-bit) | PDO (トピックデータ) — マスター割当、高速 |
| **管理層** | Extended ID (29-bit) | ディスカバリ、NMT、パラメータ、バルク |

### 4.2 管理層: Extended ID レイアウト (29-bit)

```
Bit  [28:25]  Function Code    (4 bits)
Bit  [24:18]  Source Device ID (7 bits)
Bit  [17:14]  Source Local Node (4 bits)
Bit  [13:0]   Sub-field        (14 bits)
```

| Code | 名称 | 用途 |
|------|------|------|
| 0x0 | NMT | ハートビート、状態管理 |
| 0x1 | EMCY | 緊急通知 |
| 0x2 | DISC | ディスカバリ |
| 0x3 | PARAM | パラメータ読み書き |
| 0x4 | PDO_CFG | PDO マッピング設定 |
| 0x5 | BULK | バルク転送 |
| 0x6 | SYNC | 同期信号 |
| 0x7–0xF | (予約) | 将来拡張 |

### 4.3 データ層: Standard ID (11-bit)

マスターが動的に割り当てる PDO ID。ID 値がアービトレーション優先度を決定。

```
Standard CAN ID = PDO ID (0x001–0x7FF)

割当ポリシー:
  0x001–0x0FF : 高優先度 (priority 0–1)
  0x100–0x3FF : 中優先度 (priority 2–4)
  0x400–0x7FF : 低優先度 (priority 5–7)
```

---

## 5. PDO マッピング

### 5.1 概念

マスターが全 PDO の構成を決定。ノード・トピックの境界を跨いで 1 フレームにパック可能。

```
例: PDO 0x181 — 左右エンコーダを 1 フレームに
┌─────────────────────────────────────────────────────┐
│ Motor[left]       │ Motor[left]       │ Motor[right]      │ Motor[right]      │
│ EncoderFB         │ EncoderFB         │ EncoderFB         │ EncoderFB         │
│ .position_cnt     │ .velocity_cps     │ .position_cnt     │ .velocity_cps     │
│ (4B i32)          │ (4B i32)          │ (4B i32)          │ (4B i32)          │
└─────────────────────────────────────────────────────┘
Total: 16 bytes
```

### 5.2 PDO マッピングエントリ

```
PDO Mapping Entry:
  local_node_id : uint4   — デバイス内のどのノード
  topic_index   : uint8   — ノード内のどのトピック (自動割当値)
  field_index   : uint8   — トピック内のどのフィールド (0xFF=全体)
  offset        : uint8   — PDO ペイロード内バイトオフセット
  size          : uint8   — バイトサイズ
```

### 5.3 デフォルトマッピング戦略

| 戦略 | 動作 |
|------|------|
| `one_to_one` | 1 Topic = 1 PDO |
| `pack_by_node` | 同一ノードタイプのトピックを 64B まで詰める |
| `pack_by_device` | デバイス内全ノードのトピックを 64B まで詰める |
| `custom` | YAML で明示指定 |

### 5.4 PDO 設定プロトコル

```
■ PDO_CFG_BEGIN → target device
  CAN ID: EXT_ID(PDO_CFG, master, 0, target_device_id)
  Payload:
    [0:1]  pdo_id     : uint16 LE (Standard CAN ID)
    [2]    direction   : uint8 (0=TX, 1=RX)
    [3]    num_entries : uint8
    [4:5]  period_ms   : uint16 LE (0=イベント駆動)
    [6]    total_size  : uint8

■ PDO_CFG_ENTRY × num_entries
  Payload:
    [0]  local_node_id : uint8
    [1]  topic_index   : uint8
    [2]  field_index   : uint8
    [3]  offset        : uint8
    [4]  size          : uint8

■ PDO_CFG_COMMIT
  Payload:
    [0:1]  pdo_id : uint16 LE
    [2]    action  : uint8 (0=Apply, 1=Delete)

■ PDO_CFG_ACK ← device
  Payload:
    [0:1]  pdo_id : uint16 LE
    [2]    status  : uint8 (0=OK, 1=INVALID, 2=NO_RESOURCE)
```

---

## 6. デバイス間ルーティング

マスターが TX PDO と RX PDO のペアで同じ Standard CAN ID を共有させるだけ。

```
Device A (TX PDO 0x181) ──CAN broadcast──► Device B (RX PDO 0x181)
                                          ▲
                              マスターは設定のみ、データは流れない
```

設定方法: Device A に TX PDO 0x181 を、Device B に RX PDO 0x181 を、
それぞれ PDO_CFG で設定する。専用プロトコル不要。

```yaml
routing:
  - source:
      device: "dual_motor_board_10"
      node: "left"
      topic: "encoder_feedback"
    sink:
      device: "control_board_20"
      node: "position_controller"
      topic: "position_input"
    also_publish_ros2: true
```

---

## 7. プロトコル詳細

### 7.1 ハートビート (NMT)

```
CAN ID: EXT_ID(NMT, device_id, 0, 0x0001)

Payload (最大 64 bytes):
  [0]     state             : uint8  (BOOT=0, PREOP=1, OP=2, STOPPED=3)
  [1]     num_nodes         : uint8
  [2:5]   uptime_ms         : uint32 LE
  Per node (num_nodes 回, 各 8 bytes):
    [+0:3] schema_hash      : uint32 LE  ★ ハッシュで識別
    [+4]   local_node_id    : uint8
    [+5]   instance_index   : uint8
    [+6:7] (reserved)       : uint16
```

> 例: DualMotorBoard (Motor×2 + IMU + GPIO, num_nodes=4)
> ペイロード = 6 + 4×8 = 38 bytes ≤ 64

**マスターの処理**:

1. ハートビートを受信
2. 各ノードの `schema_hash` を既知スキーマの hash テーブルで検索
3. 全ノードが既知 → コンパイル済みスキーマで即セットアップ（ディスクリプタ転送不要）
4. 未知ノードあり → ディスクリプタをバルク転送で取得

### 7.2 状態遷移

```
  BOOT ──(初期化完了)──► PREOP ──(NMT_START)──► OPERATIONAL
                          ▲                        │
                          └──(NMT_STOP)────────────┘
                          └──(NMT_RESET)──► BOOT
```

### 7.3 ディスカバリフロー

```
┌────────┐                              ┌──────────┐
│ Master │                              │  Device  │
└───┬────┘                              └─────┬────┘
    │                                         │
    │  ◄── HEARTBEAT ────────────────────────│  schema_hash ×N を含む
    │                                         │
    │  全ハッシュが既知？                        │
    │  ┌─ YES: ディスクリプタ転送スキップ         │
    │  │                                      │
    │  └─ NO:                                 │
    │── DISC_GET_DESCRIPTOR ────────────────► │
    │  ◄── BULK_TRANSFER ───────────────────│  ディスクリプタ本体
    │                                         │
    │  ※ ハッシュ照合で未知ノードのみデコード      │
    │  ※ PDO マッピング計算                      │
    │                                         │
    │── PDO_CFG ×N ─────────────────────────►│
    │  ◄── PDO_CFG_ACK ────────────────────│
    │── NMT_START ──────────────────────────►│
    │                                         │
    │  ◄══ PDO DATA (Standard ID) ══════════│
```

### 7.4 デバイスディスクリプタ

```
Device Descriptor Binary Format:
┌──────────────────────────────────────────────────────────┐
│ Header (8 bytes)                                         │
│   [0:1]  magic           = 0x50CA                        │
│   [2]    descriptor_ver  = 3                             │
│   [3]    num_nodes       : uint8                         │
│   [4:5]  total_length    : uint16 LE                     │
│   [6:7]  crc16           : uint16 LE                     │
├──────────────────────────────────────────────────────────┤
│ Device Name (length-prefixed UTF-8)                      │
├──────────────────────────────────────────────────────────┤
│ Node Descriptors × num_nodes                             │
│   Per Node:                                              │
│   ┌──────────────────────────────────────────────────┐   │
│   │ [0:3]  schema_hash     : uint32 LE    ★           │   │
│   │ [4]    local_node_id   : uint8                   │   │
│   │ [5]    instance_index  : uint8                   │   │
│   │ [6]    num_topics      : uint8                   │   │
│   │ [7]    num_params      : uint8                   │   │
│   │ [8]    name_len        : uint8                   │   │
│   │ [9..]  node_type_name  : UTF-8 (例: "BLDCMotor") │   │
│   │ [..]   instance_name   : len-prefixed UTF-8      │   │
│   ├──────────────────────────────────────────────────┤   │
│   │ Topic Descriptors × num_topics                   │   │
│   │   [0]  topic_index  : uint8                      │   │
│   │   [1]  direction    : uint8                      │   │
│   │   [2]  periodic     : uint8                      │   │
│   │   [3]  priority     : uint8                      │   │
│   │   [4]  total_size   : uint8 (packed bytes)       │   │
│   │   [5]  num_fields   : uint8                      │   │
│   │   [6]  name_len     : uint8                      │   │
│   │   [7..] topic_name  : UTF-8                      │   │
│   │   Per Field:                                     │   │
│   │     [0]  field_type : uint8                      │   │
│   │     [1]  offset     : uint8                      │   │
│   │     [2]  name_len   : uint8                      │   │
│   │     [3..] field_name: UTF-8                      │   │
│   ├──────────────────────────────────────────────────┤   │
│   │ Param Descriptors × num_params                   │   │
│   │   (Topic と同構造 + read_only : uint8)            │   │
│   └──────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────┘
```

### 7.5 パラメータアクセス

```
■ PARAM_READ_REQ
  CAN ID: EXT_ID(PARAM, requester, 0, target_dev << 4 | local_node)
  Payload: [0] 0x01 (READ), [1] param_index

■ PARAM_READ_RES
  Payload: [0] 0x81, [1] status, [2..] packed value

■ PARAM_WRITE_REQ / RES: command 0x02 / 0x82

送信周期変更: System Param 0xF0 + topic_index → uint16 period_ms
```

### 7.6 バルク転送

ISO-TP (ISO 15765-2) 準拠。ディスクリプタ・ファームウェア更新に使用。

---

## 8. ROS 2 ブリッジノード

### 8.1 自動生成トピック階層

```
/protocan/
  ├── dual_motor_board_10/
  │   ├── left/
  │   │   ├── motor_status        (Sub, protocan_msgs/MotorStatus)  ← 自動生成型
  │   │   ├── encoder_feedback    (Sub, protocan_msgs/EncoderFeedback)
  │   │   ├── cmd_vel             (Pub, geometry_msgs/msg/Twist)    ← 既存型を再利用
  │   │   └── Parameters: pid_gains, device_info, _period/*
  │   ├── right/
  │   │   └── (同上)
  │   └── imu/
  │       ├── imu_data            (Sub, sensor_msgs/msg/Imu)        ← 既存型を再利用
  │       └── Parameters: _period/imu_data_ms
  │
  └── protocan_bridge/
      ├── Parameters: can_interface, heartbeat_timeout_ms, pdo_mapping_strategy
      └── Services: list_devices, remap_pdo, add_route
```

ポイント:

- `ros2_msg_type` 指定あり → 既存 ROS 2 メッセージ型でそのまま publish/subscribe
- `ros2_msg_type` 省略 → `protocan_msgs/` 以下にカスタム `.msg` を生成して使用
- 既存型を使う場合、他の ROS 2 ノード（Nav2, MoveIt 等）と **変換なしで直結** できる

### 8.2 ブリッジ動作概要

```python
class ProtoCANBridge(Node):
    def __init__(self):
        self.can = open_canfd("can0")
        self.schema_registry = {}  # schema_hash → Schema (コンパイル済み)
        self.pdo_table = {}        # standard_id → PDO

        # コンパイル済みスキーマをハッシュテーブルに登録
        for schema in load_compiled_schemas():
            self.schema_registry[schema.hash] = schema

    def on_heartbeat(self, device_id, state, nodes_info):
        if device_id in self.active_devices:
            self.active_devices[device_id].update_heartbeat()
            return

        dev = DeviceState(device_id, nodes_info)

        # ★ スキーマハッシュで照合
        unknown_nodes = [
            n for n in nodes_info
            if n.schema_hash not in self.schema_registry
        ]

        if not unknown_nodes:
            # 全ノード既知 → 即セットアップ
            self.setup_device(dev)
        else:
            # 未知ノードあり → ディスクリプタ取得
            self.request_descriptor(dev)

    def on_descriptor_received(self, dev, descriptor):
        for node_desc in descriptor.nodes:
            if node_desc.schema_hash not in self.schema_registry:
                # ディスクリプタから動的スキーマを構築・登録
                schema = Schema.from_descriptor(node_desc)
                self.schema_registry[schema.hash] = schema
        self.setup_device(dev)

    def setup_device(self, dev):
        pdos = self.compute_pdo_mapping(dev)
        for pdo in pdos:
            self.send_pdo_config(dev.device_id, pdo)
        self.apply_routing(dev)
        self.send_nmt_start(dev.device_id)
        self.create_ros2_interface(dev)  # ★ ros2_msg_type に応じた Publisher 生成
```

### 8.3 ros2_msg_type による Publisher 生成の分岐

```python
def create_topic_publisher(self, namespace, topic_schema):
    if topic_schema.ros2_msg_type:
        # 既存 ROS 2 型を使用
        msg_class = import_ros2_msg(topic_schema.ros2_msg_type)
        # e.g. sensor_msgs.msg.Imu
        converter = build_field_converter(
            topic_schema.fields,         # proto フィールド定義
            topic_schema.ros2_field_map,  # ros2_field マッピング
            msg_class
        )
    else:
        # カスタム型を使用
        msg_class = import_ros2_msg(f"protocan_msgs/msg/{topic_schema.msg_name}")
        converter = build_direct_converter(topic_schema.fields, msg_class)

    pub = self.create_publisher(msg_class, f"{namespace}/{topic_schema.topic_name}")
    return pub, converter
```

---

## 9. コード生成

### 9.1 出力構成

```
firmware/
  protocan/                          # 共通ランタイム
    ├── protocan.h / .c
    ├── protocan_nmt.c
    ├── protocan_pdo.c
    ├── protocan_param.c
    ├── protocan_bulk.c
    └── protocan_hal.h               # HAL (ユーザー実装)

  generated/
    ├── protocan_bldc_motor.h / .c   # 構造体・定数・schema_hash
    ├── protocan_imu.h / .c
    ├── protocan_device_desc.c / .h  # ディスクリプタ (ROM)
    └── protocan_schema_hashes.h     # 全ノードのハッシュ定数

ros2_ws/src/
  protocan_msgs/msg/                 # カスタム型のみ生成
    ├── MotorStatus.msg              # ros2_msg_type 省略 → 生成
    └── EncoderFeedback.msg
    (※ ImuData, MotorCommand は sensor_msgs/Imu, geometry_msgs/Twist を使うため生成不要)

  protocan_bridge/
    ├── src/
    │   ├── bridge_node.cpp
    │   ├── schema_registry.cpp      # hash → Schema のレジストリ
    │   ├── bldc_motor_converter.cpp  # CAN ↔ ROS 2 変換 (mixed: custom + Twist)
    │   └── imu_converter.cpp         # CAN → sensor_msgs/Imu 変換
    └── config/default.yaml
```

### 9.2 生成 C コード例

```c
// ============================================================
// protocan_bldc_motor.h (自動生成)
// ============================================================
#pragma once
#include "protocan/protocan.h"

// ★ スキーマハッシュ (protoc プラグインが計算)
#define BLDC_MOTOR_SCHEMA_HASH  0xA3F7B201u

// ──── Topic Index (ファイル内出現順で自動割当) ────
#define BLDC_MOTOR_TOPIC_MOTOR_STATUS      0
#define BLDC_MOTOR_TOPIC_ENCODER_FEEDBACK  1
#define BLDC_MOTOR_TOPIC_MOTOR_COMMAND     2

// ──── Param Index ────
#define BLDC_MOTOR_PARAM_PID_GAINS         0
#define BLDC_MOTOR_PARAM_DEVICE_INFO       1

// ──── メッセージ構造体 ────
typedef struct __attribute__((packed)) {
    float    current_a;
    float    velocity_rps;
    float    temperature_c;
    uint32_t error_flags;
} protocan_motor_status_t;  // 16 bytes
_Static_assert(sizeof(protocan_motor_status_t) == 16, "");

typedef struct __attribute__((packed)) {
    int32_t position_cnt;
    int32_t velocity_cps;
} protocan_encoder_feedback_t;  // 8 bytes

typedef struct __attribute__((packed)) {
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
} protocan_motor_command_t;  // 24 bytes
// ※ ROS 2 側では geometry_msgs/Twist にマッピングされるが、
//   ファームウェア側は CAN バイナリ構造のみ意識する

typedef struct __attribute__((packed)) {
    float kp, ki, kd;
} protocan_pid_gains_t;

// ──── ノードハンドル ────
typedef struct {
    protocan_node_t              base;
    protocan_motor_status_t      motor_status;
    protocan_encoder_feedback_t  encoder_fb;
    protocan_motor_command_t     motor_cmd;
    protocan_pid_gains_t         pid_gains;

    void (*on_motor_command)(const protocan_motor_command_t*, void*);
    void *on_motor_command_ctx;
} protocan_bldc_motor_node_t;

// ──── API ────
void protocan_bldc_motor_init(protocan_bldc_motor_node_t *n, uint8_t local_node_id);

protocan_status_t protocan_bldc_motor_publish_motor_status(
    protocan_bldc_motor_node_t *n);
protocan_status_t protocan_bldc_motor_publish_encoder_feedback(
    protocan_bldc_motor_node_t *n);

void protocan_bldc_motor_on_motor_command(
    protocan_bldc_motor_node_t *n,
    void (*cb)(const protocan_motor_command_t*, void*),
    void *ctx);
```

```c
// ============================================================
// protocan_schema_hashes.h (自動生成 — デバイスマニフェストから)
// ============================================================
#pragma once

// DualMotorBoard のノード構成
#define DEVICE_NODE_COUNT  4

static const protocan_node_info_t DEVICE_NODES[] = {
    { .schema_hash = BLDC_MOTOR_SCHEMA_HASH, .local_node_id = 0, .instance_index = 0 },
    { .schema_hash = BLDC_MOTOR_SCHEMA_HASH, .local_node_id = 1, .instance_index = 1 },
    { .schema_hash = IMU_SCHEMA_HASH,        .local_node_id = 2, .instance_index = 0 },
    { .schema_hash = GPIO_SCHEMA_HASH,       .local_node_id = 3, .instance_index = 0 },
};
```

### 9.3 ファームウェア使用例

```c
#include "protocan/protocan.h"
#include "generated/protocan_bldc_motor.h"
#include "generated/protocan_imu.h"
#include "generated/protocan_device_desc.h"

static protocan_device_t device;
static protocan_bldc_motor_node_t motor_left, motor_right;
static protocan_imu_node_t imu_node;

void on_left_cmd(const protocan_motor_command_t *cmd, void *ctx) {
    // cmd->linear_x, angular_z etc. — Twist と同じ構造
    set_wheel_velocity(LEFT, cmd->linear_x, cmd->angular_z);
}

int main(void) {
    canfd_hal_init();

    protocan_device_init(&device,
        PROTOCAN_DEVICE_DESCRIPTOR, PROTOCAN_DEVICE_DESCRIPTOR_SIZE);
    protocan_device_set_id(&device, read_dip_switch());

    protocan_bldc_motor_init(&motor_left,  0);
    protocan_bldc_motor_init(&motor_right, 1);
    protocan_imu_init(&imu_node, 2);

    protocan_device_add_node(&device, &motor_left.base);
    protocan_device_add_node(&device, &motor_right.base);
    protocan_device_add_node(&device, &imu_node.base);

    protocan_bldc_motor_on_motor_command(&motor_left,  on_left_cmd,  NULL);
    protocan_bldc_motor_on_motor_command(&motor_right, on_right_cmd, NULL);

    protocan_device_start(&device);

    while (1) {
        protocan_device_poll(&device);

        if (protocan_device_state(&device) == PROTOCAN_STATE_OPERATIONAL) {
            motor_left.encoder_fb.position_cnt = read_encoder(LEFT);
            motor_left.encoder_fb.velocity_cps = read_encoder_vel(LEFT);
            // ...
            protocan_bldc_motor_publish_encoder_feedback(&motor_left);
            protocan_bldc_motor_publish_encoder_feedback(&motor_right);
            protocan_imu_publish_imu_data(&imu_node);
        }
    }
}
```

---

## 10. アドレス管理

```
Device ID (7 bits):
  0x00       : ブロードキャスト
  0x01       : マスター (PC)
  0x02–0x0F  : 予約
  0x10–0x7E  : 一般デバイス (111 台)
  0x7F       : 未割当（自動割当待ち）

Local Node ID (4 bits): 0x0–0xF — デバイス内最大 16 ノード
```

---

## 11. エラーハンドリング

| 状況 | マスター | デバイス |
|------|---------|---------|
| ハートビートタイムアウト | stale 診断、PDO 無効化 | 自律動作 or 安全停止 |
| PDO_CFG 不整合 | ディスクリプタ再取得 → リトライ | NACK |
| スキーマハッシュ衝突 | ディスクリプタの full 比較にフォールバック | — |
| CAN Bus-Off | 指数バックオフで再参加 | 同左 |

---

## 12. 設計判断のまとめ

| 項目 | v0.2 | v0.3 | 理由 |
|------|------|------|------|
| スキーマ識別 | node_type_id + schema_version | **schema_hash (FNV-1a 32-bit)** | 手動 ID 管理不要、構造変更を自動検出 |
| ROS 2 メッセージ | カスタム .msg のみ | **既存型マッピング + カスタム生成** | Nav2/MoveIt 等と変換なし直結 |
| ハートビート | node_type_id + ver で照合 | **schema_hash で O(1) 照合** | 高速ディスカバリ |
| ディスクリプタ転送 | 毎回必要 | **ハッシュ一致時はスキップ** | 起動時間短縮 |

v0.2 から継続:

- Standard ID (11-bit) データ層 / Extended ID (29-bit) 管理層
- PDO マッピングによるフレームパッキング
- TX/RX PDO ペアリングによるデバイス間ルーティング
- `periodic` フラグのみ（周期値はランタイムパラメータ）
- protoc 自動割当の topic_index / local_node_id
- device_manifest.toml によるマルチノードデバイス構成

---

## 13. 今後の検討事項

- [ ] `protoc-gen-protocan` プラグイン実装（FNV-1a ハッシュ計算含む）
- [ ] ROS 2 ブリッジ C++ 実装（schema_registry、動的コンバータ）
- [ ] `ros2_msg_type` マッピングの型互換性検証ロジック（float→double 変換等）
- [ ] PDO マッピング最適化（ビンパッキング）
- [ ] SYNC タイムスタンプ同期
- [ ] ファームウェアアップデート over CAN FD
- [ ] `Header.stamp` の高精度タイムスタンプ（CAN FD タイムスタンプ活用）
- [ ] ROS 2 既存型の逆方向マッピング（ROS 2 subscribe → CAN TX）の QoS 設計
- [ ] 複数マスター冗長構成
- [ ] `oneof` / `repeated` の将来サポート
