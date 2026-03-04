# ProtoCAN — CAN FD プラグ・アンド・プレイ通信システム仕様 (Draft v0.2)

## 1. 概要

ProtoCAN は、CAN FD バス上で複数の組み込みデバイスと PC（ROS 2）が双方向通信を行うためのプロトコルである。

### v0.1 からの主な変更

- **物理デバイスと論理ノードの分離**: 1 デバイスが複数の異種・同種ノードをホスト可能
- **Standard ID (11-bit) によるデータ転送**: マスター（PC）が割り当て、低レイテンシ
- **PDO マッピング**: 複数ノード・トピックのフィールドを 1 フレームにパック可能
- **マスター管理のルーティング**: デバイス間の直接通信をマスターが設定
- **周期はランタイムパラメータ化**: スキーマには `periodic` フラグのみ
- **チャネル ID 自動割当**: `protoc` プラグインがコンパイル時に連番付与

### 設計思想

```
CANopen の PDO マッピングモデル
 + Protobuf スキーマによるコード生成
 + ROS 2 の動的トピック生成
 = ProtoCAN
```

### 用語定義

| 用語 | 定義 |
|------|------|
| **Device** | CAN FD バス上の物理的な 1 デバイス（1 MCU / 1 CAN トランシーバ） |
| **Node** | Device 上の論理的な機能単位。1 つの `.proto` ファイルが 1 つの Node Type を定義 |
| **Node Instance** | 同一 Node Type の複数インスタンス（例: 左モーター / 右モーター） |
| **Topic** | Node が送受信するデータストリーム（ROS 2 トピックに対応） |
| **Param** | Node のランタイム設定値（ROS 2 パラメータに対応） |
| **PDO** | Process Data Object — マスターが割り当てる Standard CAN ID とフレームレイアウト |
| **Master** | PC 側の ProtoCAN ブリッジ。ID 割当・ルーティング・ディスカバリを統括 |

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
│  - packed エンコーダ      │   │  - ディスカバリ & PDO マッピング    │
│  - ディスクリプタ (ROM)    │   │  - ルーティングテーブル管理         │
│  - PDO テーブル (RAM)     │   │  - トピック/パラメータ自動生成      │
│  - プロトコルスタック      │   │  - CAN ↔ ROS 2 変換             │
└─────────┬──────────────┘   └───────────────┬────────────────┘
          │                                   │
          ▼                                   ▼
┌───────────────────────────────────────────────────────────────┐
│                        CAN FD バス                             │
│                                                               │
│  [Standard ID: データ (PDO)]     ← 高速・低オーバーヘッド        │
│  [Extended ID: 管理・ディスカバリ]  ← プラグ・アンド・プレイ        │
│                                                               │
│  Device A          Device B           PC (Master)             │
│  ├─ Node: Motor×2  ├─ Node: IMU       ProtoCAN Bridge         │
│  └─ Node: GPIO     └─ Node: GPIO                              │
└───────────────────────────────────────────────────────────────┘
```

---

## 2. スキーマ定義 (.proto)

### 2.1 カスタムオプション定義

```protobuf
// protocan/options.proto
syntax = "proto3";
package protocan;

import "google/protobuf/descriptor.proto";

// ──── ファイルレベル = Node Type 定義 ────

message NodeTypeOptions {
  // Node Type 識別子（コンパイル時に自動割当 or 明示指定）
  // 明示指定する場合: 0x0001–0xFFFF
  // 省略 (0) の場合: protoc プラグインがハッシュベースで割当
  uint32 node_type_id     = 1;
  string node_type_name   = 2;  // 人間可読名 (例: "BLDCMotor")
  uint32 schema_version   = 3;
  string ros2_namespace   = 4;  // 省略時は node_type_name から自動生成
}

extend google.protobuf.FileOptions {
  NodeTypeOptions node_type = 50000;
}

// ──── メッセージレベル = Topic / Param 定義 ────

enum Direction {
  DEVICE_TO_BUS = 0;  // デバイスが送信元
  BUS_TO_DEVICE = 1;  // デバイスが受信側
}

message TopicOptions {
  Direction direction = 1;
  bool     periodic   = 2;  // true: 周期送信（周期値はランタイムパラメータ）
                             // false: イベント駆動
  uint32   priority   = 3;  // 0=最高, 7=最低 — マスターの PDO 割当ヒント
  string   ros2_topic = 4;  // ROS 2 トピック名オーバーライド
}

message ParamOptions {
  bool   read_only   = 1;
  string ros2_param  = 2;  // ROS 2 パラメータ名オーバーライド
}

extend google.protobuf.MessageOptions {
  TopicOptions topic = 50001;
  ParamOptions param = 50002;
}

// ──── フィールドレベル ────
// フィールドは proto の field number 順にソートされ、
// コンパイル時に topic 内の offset が確定する。
// field number は packed layout の物理順序を決定するため、
// **一度リリースしたら変更不可**（Protobuf の互換性ルールと同じ）。
```

### 2.2 Node Type 定義例: BLDC モータードライバ

```protobuf
// bldc_motor.proto
syntax = "proto3";
package bldc_motor;

import "protocan/options.proto";

option (protocan.node_type) = {
  node_type_id: 0x0010,
  node_type_name: "BLDCMotor",
  schema_version: 1,
  ros2_namespace: "motor"
};

// ──── Topics ────

// topic_index は protoc プラグインが自動割当（ファイル内の message 出現順）
// → この例では MotorStatus=0, EncoderFeedback=1, MotorCommand=2

message MotorStatus {
  option (protocan.topic) = {
    direction: DEVICE_TO_BUS,
    periodic: true,
    priority: 3
  };
  // field number 順に packed: 合計 16 bytes
  float  current_a     = 1;  // 4B
  float  velocity_rps  = 2;  // 4B
  float  temperature_c = 3;  // 4B
  uint32 error_flags   = 4;  // 4B
}

message EncoderFeedback {
  option (protocan.topic) = {
    direction: DEVICE_TO_BUS,
    periodic: true,
    priority: 1
  };
  // 合計 8 bytes
  int32 position_cnt = 1;  // 4B
  int32 velocity_cps = 2;  // 4B
}

message MotorCommand {
  option (protocan.topic) = {
    direction: BUS_TO_DEVICE,
    periodic: false,
    priority: 2
  };
  // 合計 12 bytes
  float  target_velocity_rps = 1;
  float  torque_limit_a      = 2;
  uint32 control_mode        = 3;
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

### 2.3 Node Type 定義例: IMU

```protobuf
// imu.proto
syntax = "proto3";
package imu;

import "protocan/options.proto";

option (protocan.node_type) = {
  node_type_id: 0x0020,
  node_type_name: "IMU",
  schema_version: 1,
  ros2_namespace: "imu"
};

message ImuData {
  option (protocan.topic) = {
    direction: DEVICE_TO_BUS,
    periodic: true,
    priority: 1
  };
  // 合計 24 bytes
  float accel_x = 1;
  float accel_y = 2;
  float accel_z = 3;
  float gyro_x  = 4;
  float gyro_y  = 5;
  float gyro_z  = 6;
}
```

### 2.4 マルチノードデバイスの構成

1 つの物理デバイスが複数の Node を持つ場合、**デバイスマニフェスト** で宣言する。
`.proto` ファイル自体は単一ノードタイプの定義に留める（関心の分離）。

```toml
# device_manifest.toml — ファームウェアビルド時の設定ファイル
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

protoc プラグインはこのマニフェストに基づき:

1. 各 Node Instance に **local_node_id** を自動割当:
   - `bldc_motor[0] ("left")  → local_node_id = 0`
   - `bldc_motor[1] ("right") → local_node_id = 1`
   - `imu[0]                  → local_node_id = 2`
   - `gpio[0]                 → local_node_id = 3`
2. デバイスディスクリプタを生成
3. 全ノードの C コードをまとめてリンク

### 2.5 型マッピング（Proto型 → バイナリ表現）

| Proto 型      | CAN バイナリ          | サイズ   |
|--------------|--------------------|---------|
| `bool`       | `uint8` (0 or 1)   | 1 byte  |
| `uint32`     | little-endian u32  | 4 bytes |
| `int32`      | little-endian i32  | 4 bytes |
| `float`      | IEEE 754 LE        | 4 bytes |
| `double`     | IEEE 754 LE        | 8 bytes |
| `uint64`     | little-endian u64  | 8 bytes |
| `int64`      | little-endian i64  | 8 bytes |

> 可変長フィールド (`string`, `bytes`, `repeated`) は v0.2 ではサポート外。
> `oneof` もサポート外。将来バージョンで検討。

---

## 3. CAN フレーム設計

### 3.1 二層構成

| 層 | CAN ID 形式 | 用途 | 特徴 |
|----|------------|------|------|
| **データ層** | Standard ID (11-bit) | PDO (トピックデータ) | 低オーバーヘッド・高速、マスター割当 |
| **管理層** | Extended ID (29-bit) | ディスカバリ、NMT、パラメータ、バルク | プラグ・アンド・プレイ、自己記述 |

Standard ID フレームはヘッダが短くスループットが高い。
データ転送は Standard ID、管理系のみ Extended ID を使うことで帯域を最大化。

### 3.2 管理層: Extended ID レイアウト (29-bit)

```
Bit  [28:25]  Function Code    (4 bits)
Bit  [24:18]  Source Device ID (7 bits)  物理デバイス識別
Bit  [17:14]  Source Local Node (4 bits) デバイス内ノード (0–15)
Bit  [13:0]   Sub-field        (14 bits) コンテキスト依存
```

#### Function Code 一覧（管理層）

| Code | 名称      | 方向         | 用途                                    |
|------|-----------|-----------  |-----------------------------------------|
| 0x0  | NMT       | 双方向       | ハートビート、状態管理、START/STOP          |
| 0x1  | EMCY      | デバイス→全体 | 緊急通知（障害・異常）                      |
| 0x2  | DISC      | 双方向       | ディスカバリプロトコル                      |
| 0x3  | PARAM     | 双方向       | パラメータ読み書き                         |
| 0x4  | PDO_CFG   | マスター→デバ | PDO マッピング設定                         |
| 0x5  | ROUTE_CFG | マスター→デバ | ルーティングテーブル設定                    |
| 0x6  | BULK      | 双方向       | バルク転送（ディスクリプタ、FW更新）         |
| 0x7  | SYNC      | マスター→全体 | 同期信号                                  |
| 0x8–0xF | (予約) |             | 将来拡張                                  |

### 3.3 データ層: Standard ID (11-bit)

Standard ID はマスターが動的に割り当てる **PDO ID**。
ID の値そのものが CAN アービトレーション優先度を決定する（小さい = 高優先度）。

```
Standard CAN ID = PDO ID (0x001–0x7FF)
```

マスターの割当ポリシー:

```
  0x001–0x0FF : 高優先度 PDO (priority 0–1)  — 256 個
  0x100–0x3FF : 中優先度 PDO (priority 2–4)  — 768 個
  0x400–0x7FF : 低優先度 PDO (priority 5–7)  — 1024 個

  ※ 0x000 は CAN 仕様上使用不可
```

---

## 4. PDO マッピング

### 4.1 概念

CANopen の PDO マッピングに相当。マスターが全 PDO の構成を決定する。

**1 フレームにできるだけ詰め込む** — ノード・トピックの境界を跨いでパック可能。

```
例: PDO ID 0x181 (Device 0x10 が送信)
┌─────────────────────────────────────────────────────┐
│ Offset 0   │ Offset 4   │ Offset 8   │ Offset 12  │
│ Motor[0]   │ Motor[0]   │ Motor[1]   │ Motor[1]   │
│ Encoder    │ Encoder    │ Encoder    │ Encoder    │
│ .position  │ .velocity  │ .position  │ .velocity  │
│ (4B i32)   │ (4B i32)   │ (4B i32)   │ (4B i32)   │
└─────────────────────────────────────────────────────┘
Total: 16 bytes — 左右エンコーダを 1 フレームにパック
```

```
例: PDO ID 0x182 (Device 0x10 が送信)
┌───────────────────────────────────────────────────────────┐
│ Motor[0].Status              │ Motor[1].Status             │
│ .current .velocity .temp .err│ .current .velocity .temp .err│
│ (4B)(4B)(4B)(4B)             │ (4B)(4B)(4B)(4B)            │
└───────────────────────────────────────────────────────────┘
Total: 32 bytes — 左右モーターステータスを 1 フレームにパック
```

### 4.2 PDO マッピングエントリ

```
PDO Mapping Entry:
  local_node_id   : uint4   — デバイス内のどのノード
  topic_index     : uint8   — そのノード内のどのトピック (自動割当値)
  field_index     : uint8   — そのトピック内のどのフィールド (0xFF=全フィールド)
  offset          : uint8   — PDO ペイロード内のバイトオフセット
  size            : uint8   — バイトサイズ
```

### 4.3 デフォルトマッピング戦略

マスターは以下のポリシーを選択できる:

| 戦略 | 動作 |
|------|------|
| `one_to_one` | 1 Topic = 1 PDO（シンプル、デバッグ向き） |
| `pack_by_node` | 同一ノードタイプのトピックを 64B まで詰める |
| `pack_by_device` | デバイス内の全ノードのトピックを 64B まで詰める |
| `custom` | YAML で明示指定 |

```yaml
# protocan_bridge 設定例
pdo_mapping:
  strategy: "pack_by_device"

  custom:
    - pdo_id: 0x181
      transmitter: { device_id: 0x10 }
      priority: 1
      entries:
        - { node: "left/encoder_feedback" }
        - { node: "right/encoder_feedback" }
    - pdo_id: 0x280
      transmitter: { device_id: 0x10 }
      priority: 3
      entries:
        - { node: "left/motor_status", fields: ["current_a", "velocity_rps"] }
        - { node: "right/motor_status", fields: ["current_a", "velocity_rps"] }
```

### 4.4 PDO 設定プロトコル

マスターがデバイスに PDO マッピングを書き込む:

```
■ PDO_CFG_BEGIN
  CAN ID: EXT_ID(PDO_CFG, master_device, 0, target_device_id)
  Payload:
    [0:1]  pdo_id         : uint16 LE (Standard CAN ID)
    [2]    direction       : uint8 (0=TX from device, 1=RX to device)
    [3]    num_entries     : uint8
    [4:5]  period_ms       : uint16 LE (TX周期, 0=イベント駆動)
    [6]    total_size      : uint8 (PDO ペイロードサイズ)

■ PDO_CFG_ENTRY (num_entries 回送信)
  CAN ID: EXT_ID(PDO_CFG, master_device, 0, sequence_num)
  Payload:
    [0]    local_node_id   : uint8
    [1]    topic_index     : uint8
    [2]    field_index     : uint8  (0xFF = トピック全フィールド)
    [3]    offset          : uint8
    [4]    size            : uint8

■ PDO_CFG_COMMIT
  CAN ID: EXT_ID(PDO_CFG, master_device, 0, 0x3FFF)
  Payload:
    [0:1]  pdo_id         : uint16 LE
    [2]    action          : uint8 (0=Apply, 1=Delete)

■ PDO_CFG_ACK (デバイス → マスター)
  CAN ID: EXT_ID(PDO_CFG, device_id, 0, master_device_id)
  Payload:
    [0:1]  pdo_id         : uint16 LE
    [2]    status          : uint8 (0=OK, 1=INVALID, 2=NO_RESOURCE)
```

---

## 5. デバイス間ルーティング

### 5.1 概念

マスターが「Device A の TX PDO を Device B の RX PDO として購読させる」ことで、
PC を介さずにデバイス間直接通信を実現する。

```
┌──────────────┐                    ┌──────────────┐
│  Device A    │                    │  Device B    │
│  Motor[0]    │  PDO 0x181        │  Controller  │
│  encoder_fb  ├───────────────────►│  position_in │
│  (TX)        │  Standard ID      │  (RX)        │
└──────────────┘  CAN broadcast    └──────────────┘
                       ↑
                  マスターは設定のみ、データは流れない
```

仕組み:

1. マスターが Device A に TX PDO 0x181 を設定
2. マスターが Device B に RX PDO 0x181 を設定（同じ Standard CAN ID を受信）
3. Device B は CAN ハードウェアフィルタで 0x181 を受信 → 内部トピックにデマップ

### 5.2 ルーティング設定

Device B への RX PDO 設定は **PDO_CFG そのもの** で行う:

```
PDO_CFG_BEGIN to Device B:
  pdo_id    = 0x181
  direction = 1 (RX)
  ...

PDO_CFG_ENTRY:
  local_node_id = 0 (Device B の position_controller ノード)
  topic_index   = 0 (position_input トピック)
  field_index   = 0xFF (全フィールド)
  offset        = 0
  size          = 8
```

つまり、**TX PDO と RX PDO のペアで同じ Standard CAN ID を共有** させるだけ。
追加のルーティングプロトコルは不要 — PDO_CFG の direction フラグで区別する。

### 5.3 ROS 2 側でのルーティング設定

```yaml
# protocan_bridge 設定
routing:
  - source:
      device: "dual_motor_board_10"
      node: "left"
      topic: "encoder_feedback"
    sink:
      device: "control_board_20"
      node: "position_controller"
      topic: "position_input"
    also_publish_ros2: true  # PC でも受信する場合
```

ブリッジはデバイス接続時にこの設定を読み:

1. Device A に TX PDO を設定
2. Device B に同じ PDO ID の RX PDO を設定
3. `also_publish_ros2: true` なら PC 側でも同じ Standard CAN ID をフィルタ

---

## 6. プロトコル詳細

### 6.1 ノード管理 (NMT)

#### ハートビート

デバイス単位で周期送信（デフォルト 500ms）。ホストする全ノードの一覧を含む。

```
CAN ID: EXT_ID(NMT, device_id, 0, 0x0001)

Payload (最大 64 bytes):
  [0]     state             : uint8  (BOOT=0, PREOP=1, OP=2, STOPPED=3)
  [1]     num_nodes         : uint8
  [2:5]   uptime_ms         : uint32 LE
  Per node (num_nodes 回, 各 5 bytes):
    [+0:1] node_type_id     : uint16 LE
    [+2]   local_node_id    : uint8
    [+3]   schema_version   : uint8
    [+4]   instance_index   : uint8
```

> 例: DualMotorBoard (Motor×2 + IMU + GPIO, num_nodes=4)
> ペイロード = 6 + 4×5 = 26 bytes ≤ 64

#### 状態遷移

```
  BOOT ──(初期化完了)──► PREOP ──(NMT_START)──► OPERATIONAL
                          ▲                        │
                          └──(NMT_STOP)────────────┘
                          │
                          └──(NMT_RESET)──► BOOT
```

- **BOOT**: 起動直後。ディスクリプタ応答のみ可能。
- **PREOP**: PDO マッピング設定を受付。データ送受信は停止。
- **OPERATIONAL**: PDO テーブルに従いデータ送受信を実行。

### 6.2 ディスカバリフロー

```
┌────────┐                              ┌──────────┐
│ Master │                              │  Device  │
└───┬────┘                              └─────┬────┘
    │                                         │
    │       ◄── HEARTBEAT (state=BOOT) ──────│  ノード一覧を含む
    │                                         │
    │  ※ 新規デバイス検出                        │
    │                                         │
    │── DISC_GET_DESCRIPTOR ────────────────► │
    │       ◄── BULK_TRANSFER ───────────────│  ディスクリプタ本体
    │                                         │
    │  ※ ディスクリプタ解析                      │
    │  ※ PDO マッピング計算                      │
    │                                         │
    │── PDO_CFG_BEGIN ───────────────────────►│─┐
    │── PDO_CFG_ENTRY ×N ───────────────────►│ │ PDO 設定
    │── PDO_CFG_COMMIT ─────────────────────►│─┘
    │       ◄── PDO_CFG_ACK ────────────────│
    │                                         │
    │── NMT_START ──────────────────────────►│  → OPERATIONAL
    │                                         │
    │  ※ ROS 2 トピック/パラメータ生成            │
    │                                         │
    │       ◄══ PDO DATA (Standard ID) ══════│  データ転送開始
```

### 6.3 デバイスディスクリプタ

protoc プラグインが `.proto` + `device_manifest.toml` から ROM データとして生成。

```
Device Descriptor Binary Format:
┌──────────────────────────────────────────────────────────┐
│ Header (8 bytes)                                         │
│   [0:1]   magic           = 0x50CA                       │
│   [2]     descriptor_ver  = 2                            │
│   [3]     num_nodes       : uint8                        │
│   [4:5]   total_length    : uint16 LE                    │
│   [6:7]   crc16           : uint16 LE                    │
├──────────────────────────────────────────────────────────┤
│ Device Name (length-prefixed UTF-8)                      │
│   [0]     name_len : uint8                               │
│   [1..]   name     : UTF-8                               │
├──────────────────────────────────────────────────────────┤
│ Node Descriptors × num_nodes                             │
│   Per Node:                                              │
│   ┌──────────────────────────────────────────────────┐   │
│   │ [0:1]   node_type_id    : uint16 LE              │   │
│   │ [2]     local_node_id   : uint8                  │   │
│   │ [3]     schema_version  : uint8                  │   │
│   │ [4]     instance_index  : uint8                  │   │
│   │ [5]     num_topics      : uint8                  │   │
│   │ [6]     num_params      : uint8                  │   │
│   │ [7]     name_len        : uint8                  │   │
│   │ [8..]   instance_name   : UTF-8 (例: "left")     │   │
│   ├──────────────────────────────────────────────────┤   │
│   │ Topic Descriptors × num_topics                   │   │
│   │   [0]  topic_index   : uint8 (自動割当)           │   │
│   │   [1]  direction     : uint8                     │   │
│   │   [2]  periodic      : uint8 (bool)              │   │
│   │   [3]  priority      : uint8                     │   │
│   │   [4]  total_size    : uint8 (packed bytes)      │   │
│   │   [5]  num_fields    : uint8                     │   │
│   │   [6]  name_len      : uint8                     │   │
│   │   [7..] topic_name   : UTF-8                     │   │
│   │   Per Field:                                     │   │
│   │     [0]  field_type  : uint8 (型 enum)           │   │
│   │     [1]  offset      : uint8 (packed 内位置)     │   │
│   │     [2]  name_len    : uint8                     │   │
│   │     [3..] field_name : UTF-8                     │   │
│   ├──────────────────────────────────────────────────┤   │
│   │ Param Descriptors × num_params                   │   │
│   │   (Topic と同構造 + read_only : uint8)            │   │
│   └──────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────┘

Field Type Enum:
  0x01=bool  0x02=uint8  0x03=int8  0x04=uint16  0x05=int16
  0x06=uint32  0x07=int32  0x08=uint64  0x09=int64
  0x0A=float32  0x0B=float64
```

### 6.4 パラメータアクセス

```
■ PARAM_READ_REQ
  CAN ID: EXT_ID(PARAM, requester, 0,
                 target_device_id << 4 | local_node_id)
  Payload:
    [0]  command     = 0x01 (READ)
    [1]  param_index : uint8

■ PARAM_READ_RES
  CAN ID: EXT_ID(PARAM, responder, local_node_id, requester_device_id)
  Payload:
    [0]  command = 0x81
    [1]  status  : uint8 (0=OK, 1=NOT_FOUND, 2=ERROR)
    [2..] value  : packed binary

■ PARAM_WRITE_REQ / RES: command = 0x02 / 0x82 (同様の構造)
```

**送信周期の変更**: 各 periodic トピックに暗黙のシステムパラメータを自動生成:

```
System Param Index = 0xF0 + topic_index:
  uint16 period_ms  — 0 で停止
```

### 6.5 バルク転送

ISO-TP (ISO 15765-2) 準拠:

```
CAN ID: EXT_ID(BULK, source_device, 0, transfer_context)

First Frame:     [0:3] total_length (u32 LE), [4..] data
Consecutive:     [0] sequence (u8), [1..] data
Flow Control:    [0] fc_flag, [1] block_size, [2:3] st_min_us
```

---

## 7. ROS 2 ブリッジノード

### 7.1 自動生成されるトピック階層

```
/protocan/
  ├── dual_motor_board_10/              # {device_type_name}_{device_id_hex}
  │   ├── left/                         # instance_name
  │   │   ├── motor_status       (Sub)  # ← DEVICE_TO_BUS
  │   │   ├── encoder_feedback   (Sub)
  │   │   ├── motor_command      (Pub)  # → BUS_TO_DEVICE
  │   │   └── Parameters:
  │   │       ├── pid_gains             # R/W
  │   │       ├── device_info           # R
  │   │       └── _period/
  │   │           ├── motor_status_ms
  │   │           └── encoder_feedback_ms
  │   ├── right/
  │   │   └── (同上)
  │   └── imu/
  │       ├── imu_data           (Sub)
  │       └── Parameters:
  │           └── _period/imu_data_ms
  │
  └── protocan_bridge/
      ├── Parameters:
      │   ├── can_interface          # "can0"
      │   ├── heartbeat_timeout_ms   # 2000
      │   └── pdo_mapping_strategy   # "pack_by_device"
      └── Services:
          ├── list_devices
          ├── remap_pdo
          └── add_route
```

### 7.2 ブリッジ動作概要

```python
class ProtoCANBridge(Node):
    def __init__(self):
        self.can = open_canfd("can0")
        self.known_schemas = load_compiled_schemas()
        self.pdo_table = {}       # standard_can_id → PDO 定義
        self.active_devices = {}
        self.routing = load_routing_config()

    def on_heartbeat(self, device_id, state, nodes_info):
        if device_id not in self.active_devices:
            dev = DeviceState(device_id, nodes_info)
            if all_schemas_known(nodes_info):
                self.setup_device(dev)
            else:
                self.request_descriptor(dev)

    def setup_device(self, dev):
        pdos = self.compute_pdo_mapping(dev)      # マッピング計算
        for pdo in pdos:
            self.send_pdo_config(dev.device_id, pdo)  # デバイスに設定
        for route in self.routing:
            if route.involves(dev):
                self.setup_route(route)            # ルーティング設定
        self.send_nmt_start(dev.device_id)         # OPERATIONAL へ
        self.create_ros2_interface(dev)             # ROS 2 トピック生成
        for pdo in pdos:
            self.can.set_filter(pdo.standard_id)   # 受信フィルタ
```

---

## 8. コード生成

### 8.1 出力構成

```
firmware/
  protocan/                          # 共通ランタイム
    ├── protocan.h / .c              # コア API
    ├── protocan_nmt.c               # ハートビート・状態管理
    ├── protocan_pdo.c               # PDO テーブル管理
    ├── protocan_param.c             # パラメータアクセス
    ├── protocan_bulk.c              # ISO-TP バルク転送
    └── protocan_hal.h               # HAL (ユーザー実装)

  generated/                         # protoc プラグイン生成物
    ├── protocan_bldc_motor.h / .c   # 構造体・エンコーダ
    ├── protocan_imu.h / .c
    ├── protocan_device_desc.c / .h  # ディスクリプタ (const ROM)
    └── protocan_device_nodes.h      # local_node_id 定数

ros2_ws/src/
  protocan_msgs/msg/                 # ROS 2 メッセージ型
    ├── MotorStatus.msg
    ├── EncoderFeedback.msg
    └── ImuData.msg
  protocan_bridge/                   # ブリッジノード
    ├── src/bridge_node.cpp
    └── config/default.yaml
```

### 8.2 生成 C コード例

```c
// ============================================================
// protocan_bldc_motor.h (自動生成)
// ============================================================
#pragma once
#include "protocan/protocan.h"

#define BLDC_MOTOR_NODE_TYPE_ID     0x0010
#define BLDC_MOTOR_SCHEMA_VERSION   1

// ──── Topic Index (自動割当: message 出現順) ────
#define BLDC_MOTOR_TOPIC_MOTOR_STATUS      0
#define BLDC_MOTOR_TOPIC_ENCODER_FEEDBACK  1
#define BLDC_MOTOR_TOPIC_MOTOR_COMMAND     2

// ──── Param Index (自動割当) ────
#define BLDC_MOTOR_PARAM_PID_GAINS         0
#define BLDC_MOTOR_PARAM_DEVICE_INFO       1

// ──── メッセージ構造体 ────
typedef struct __attribute__((packed)) {
    float    current_a;       // offset 0, 4B
    float    velocity_rps;    // offset 4, 4B
    float    temperature_c;   // offset 8, 4B
    uint32_t error_flags;     // offset 12, 4B
} protocan_motor_status_t;    // total: 16 bytes
_Static_assert(sizeof(protocan_motor_status_t) == 16, "");

typedef struct __attribute__((packed)) {
    int32_t position_cnt;
    int32_t velocity_cps;
} protocan_encoder_feedback_t;  // total: 8 bytes
_Static_assert(sizeof(protocan_encoder_feedback_t) == 8, "");

typedef struct __attribute__((packed)) {
    float    target_velocity_rps;
    float    torque_limit_a;
    uint32_t control_mode;
} protocan_motor_command_t;     // total: 12 bytes
_Static_assert(sizeof(protocan_motor_command_t) == 12, "");

typedef struct __attribute__((packed)) {
    float kp, ki, kd;
} protocan_pid_gains_t;

// ──── ノードハンドル ────
typedef struct {
    protocan_node_t              base;
    protocan_motor_status_t      motor_status;    // TX バッファ
    protocan_encoder_feedback_t  encoder_fb;      // TX バッファ
    protocan_motor_command_t     motor_cmd;        // RX バッファ

    protocan_pid_gains_t         pid_gains;        // パラメータ

    void (*on_motor_command)(const protocan_motor_command_t*, void*);
    void *on_motor_command_ctx;
} protocan_bldc_motor_node_t;

// ──── API ────
void protocan_bldc_motor_init(protocan_bldc_motor_node_t *n, uint8_t local_node_id);

// Publish: 構造体を埋めた後に呼ぶ
// → PDO テーブルを参照し、割り当てられた Standard ID フレームに pack して送信
protocan_status_t protocan_bldc_motor_publish_motor_status(
    protocan_bldc_motor_node_t *n);

protocan_status_t protocan_bldc_motor_publish_encoder_feedback(
    protocan_bldc_motor_node_t *n);

// Subscribe コールバック
void protocan_bldc_motor_on_motor_command(
    protocan_bldc_motor_node_t *n,
    void (*cb)(const protocan_motor_command_t*, void*),
    void *ctx);
```

### 8.3 ファームウェア使用例: DualMotorBoard

```c
#include "protocan/protocan.h"
#include "generated/protocan_bldc_motor.h"
#include "generated/protocan_imu.h"
#include "generated/protocan_device_desc.h"

static protocan_device_t device;

static protocan_bldc_motor_node_t motor_left;
static protocan_bldc_motor_node_t motor_right;
static protocan_imu_node_t        imu_node;

void on_left_cmd(const protocan_motor_command_t *cmd, void *ctx) {
    motor_set_target(MOTOR_LEFT, cmd->target_velocity_rps);
}
void on_right_cmd(const protocan_motor_command_t *cmd, void *ctx) {
    motor_set_target(MOTOR_RIGHT, cmd->target_velocity_rps);
}

int main(void) {
    canfd_hal_init();

    // デバイス初期化（ディスクリプタは const ROM）
    protocan_device_init(&device,
        PROTOCAN_DEVICE_DESCRIPTOR, PROTOCAN_DEVICE_DESCRIPTOR_SIZE);
    protocan_device_set_id(&device, read_dip_switch());

    // ノード初期化 (local_node_id はマニフェストの割当)
    protocan_bldc_motor_init(&motor_left,  0);
    protocan_bldc_motor_init(&motor_right, 1);
    protocan_imu_init(&imu_node, 2);

    // デバイスにノードを登録
    protocan_device_add_node(&device, &motor_left.base);
    protocan_device_add_node(&device, &motor_right.base);
    protocan_device_add_node(&device, &imu_node.base);

    // コールバック登録
    protocan_bldc_motor_on_motor_command(&motor_left,  on_left_cmd,  NULL);
    protocan_bldc_motor_on_motor_command(&motor_right, on_right_cmd, NULL);

    // 開始 (BOOT → PREOP → マスターの設定待ち)
    protocan_device_start(&device);

    while (1) {
        protocan_device_poll(&device);  // 受信・HB・タイマー

        if (protocan_device_state(&device) == PROTOCAN_STATE_OPERATIONAL) {
            // センサー読み取り → 構造体に書き込み
            motor_left.motor_status.current_a    = read_current(LEFT);
            motor_left.motor_status.velocity_rps = read_velocity(LEFT);
            motor_left.motor_status.temperature_c = read_temp(LEFT);
            motor_left.motor_status.error_flags  = get_errors(LEFT);

            motor_left.encoder_fb.position_cnt = read_encoder(LEFT);
            motor_left.encoder_fb.velocity_cps = read_encoder_vel(LEFT);

            // (right, imu 同様)

            // Publish — PDO テーブルに従い自動 pack & 送信
            protocan_bldc_motor_publish_motor_status(&motor_left);
            protocan_bldc_motor_publish_motor_status(&motor_right);
            protocan_bldc_motor_publish_encoder_feedback(&motor_left);
            protocan_bldc_motor_publish_encoder_feedback(&motor_right);
            protocan_imu_publish_imu_data(&imu_node);
        }
    }
}
```

### 8.4 PDO エンジン内部

```c
// protocan_pdo.c — publish 時の処理

protocan_status_t protocan_pdo_publish(
    protocan_device_t *dev,
    uint8_t local_node_id,
    uint8_t topic_index,
    const void *data, uint8_t data_size)
{
    for (int i = 0; i < dev->num_tx_pdos; i++) {
        protocan_pdo_t *pdo = &dev->tx_pdos[i];
        for (int j = 0; j < pdo->num_entries; j++) {
            protocan_pdo_entry_t *e = &pdo->entries[j];
            if (e->local_node_id != local_node_id ||
                e->topic_index != topic_index) continue;

            if (e->field_index == 0xFF) {
                // トピック全体をコピー
                memcpy(&pdo->buf[e->offset], data, data_size);
            } else {
                // 特定フィールドのみ
                memcpy(&pdo->buf[e->offset],
                       (const uint8_t*)data + e->field_src_offset,
                       e->size);
            }
            pdo->dirty = true;
        }

        // イベント駆動 PDO: dirty なら即送信
        if (pdo->dirty && !pdo->periodic) {
            canfd_send(pdo->standard_can_id, pdo->buf, pdo->total_size);
            pdo->dirty = false;
        }
        // periodic PDO はタイマー割り込みで送信
    }
    return PROTOCAN_OK;
}
```

---

## 9. アドレス管理

### 9.1 アドレス空間

```
Device ID (7 bits):
  0x00       : ブロードキャスト
  0x01       : マスター (PC) ※デフォルト
  0x02–0x0F  : 予約（追加マスター・ゲートウェイ）
  0x10–0x7E  : 一般デバイス (111 台)
  0x7F       : 未割当（自動割当待ち）

Local Node ID (4 bits):
  0x0–0xF    : デバイス内最大 16 ノード (マニフェストで自動割当)
```

### 9.2 Device ID 割当

| 方式 | 設定 | ユースケース |
|------|------|-------------|
| 固定 | DIP スイッチ / Flash | 本番 |
| 自動 | device_id=0x7F で起動 → マスターが割当 | プロトタイプ |

---

## 10. エラーハンドリング

| 状況 | マスター側対応 | デバイス側対応 |
|------|-------------|-------------|
| ハートビートタイムアウト | ROS 2 に stale 診断、PDO 無効化 | 自律動作 or 安全停止 |
| PDO_CFG 不整合 | ディスクリプタ再取得 → リトライ | NACK を返す |
| スキーマバージョン不一致 | ディスクリプタから動的デコーダ構築 | — |
| CAN Bus-Off | 指数バックオフで再参加 | 同左 |

---

## 11. 設計判断のまとめ

| 項目 | v0.1 | v0.2 | 理由 |
|------|------|------|------|
| デバイスとノード | 1:1 | 1:N (マニフェスト) | マルチノードデバイス対応 |
| データ CAN ID | Extended (29-bit) | **Standard (11-bit)** | ヘッダ短縮、マスター割当 |
| 優先度の表現 | Function Code 分割 | **Standard ID の値** | CAN アービトレーションに直結 |
| フレームパッキング | 1 Topic = 1 Frame | **PDO マッピング** | 64B 最大活用 |
| 周期指定 | proto に period_ms | **periodic フラグのみ** | ランタイム柔軟性 |
| channel_id | proto に明示 | **protoc 自動割当** | 人為ミス排除 |
| デバイス間通信 | CAN フィルタ直接設定 | **マスター管理 PDO** | 統一管理、可視性 |
| ルーティング | 専用プロトコル | **PDO_CFG の TX/RX ペア** | シンプル化 |

---

## 12. 今後の検討事項

- [ ] `protoc-gen-protocan` プラグイン実装
- [ ] ROS 2 ブリッジ C++ 実装
- [ ] PDO マッピング最適化アルゴリズム（ビンパッキング問題）
- [ ] SYNC タイムスタンプ同期の詳細設計
- [ ] ファームウェアアップデート over CAN FD
- [ ] CAN FD ↔ Ethernet ゲートウェイ
- [ ] 複数マスター冗長構成
- [ ] PDO マッピングの永続化（デバイス Flash 保存）
- [ ] `oneof` / `repeated` の将来サポート
- [ ] PDO マッピング変更時の OPERATIONAL → PREOP → OPERATIONAL 遷移
