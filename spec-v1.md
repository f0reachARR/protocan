# ProtoCAN — CAN FD プラグ・アンド・プレイ通信システム仕様 (Draft v0.1)

## 1. 概要

ProtoCAN は、CAN FD バス上で複数の組み込みデバイスと PC（ROS 2）が双方向通信を行うためのプロトコルである。

### 設計目標

- **プラグ・アンド・プレイ**: デバイス接続時に PC 側が自動で ROS 2 トピック・パラメータを生成
- **低オーバーヘッド**: データフレームにタグやメタデータを含まない固定レイアウトエンコーディング
- **双方向・デバイス間通信**: CAN のブロードキャスト特性を活かし、PC 経由なしでデバイス同士も直接通信
- **スキーマ駆動**: `.proto` ファイルからファームウェア用 C ライブラリと ROS 2 ブリッジコードを自動生成
- **自己記述デバイス**: 各デバイスがコンパクトなディスクリプタを内蔵し、接続時に自己宣言

### アーキテクチャ全体像

```
┌──────────────────────────────────────────────────────────┐
│  .proto スキーマ定義 (カスタムオプション付き)                   │
└──────────┬───────────────────────────┬───────────────────┘
           │ protoc プラグイン           │ protoc プラグイン
           ▼                           ▼
┌─────────────────────┐    ┌─────────────────────────────┐
│  ファームウェア用        │    │  ROS 2 ブリッジノード          │
│  C ライブラリ           │    │  (自動生成)                   │
│  - エンコーダ/デコーダ    │    │  - CAN ↔ ROS 2 変換          │
│  - ディスクリプタ (ROM)  │    │  - ディスカバリ処理             │
│  - プロトコルスタック     │    │  - トピック/パラメータ自動生成   │
└────────┬────────────┘    └──────────────┬──────────────┘
         │                                │
         ▼                                ▼
┌──────────────────────────────────────────────────────────┐
│                      CAN FD バス                          │
│  デバイスA ◄──►  デバイスB ◄──►  デバイスC ◄──► PC (ROS 2) │
└──────────────────────────────────────────────────────────┘
```

---

## 2. スキーマ定義 (.proto 拡張)

標準の Protocol Buffers 構文に **カスタムオプション** を追加し、CAN FD 通信に必要なメタデータを記述する。
`protoc` の正規構文内に収まるため、既存ツールチェーンと互換性を保つ。

### 2.1 オプション定義ファイル

```protobuf
// protocan_options.proto
syntax = "proto3";
package protocan;

import "google/protobuf/descriptor.proto";

// ──────────── ファイルレベルオプション ────────────

message DeviceOptions {
  uint32 device_type_id  = 1;  // デバイス種別 ID（0x0001–0xFFFF）
  string device_type_name = 2; // 人間可読な名前
  uint32 schema_version  = 3;  // スキーマバージョン
  string ros2_namespace  = 4;  // ROS 2 名前空間のプレフィックス（省略時は自動）
}

extend google.protobuf.FileOptions {
  DeviceOptions device = 50000;
}

// ──────────── メッセージレベルオプション ────────────

enum Direction {
  PUBLISH   = 0;  // デバイス → バス（デバイスが送信元）
  SUBSCRIBE = 1;  // バス → デバイス（デバイスが受信側）
}

message TopicOptions {
  Direction direction  = 1;
  uint32 channel_id    = 2;  // チャネル番号（ファイル内で一意、0 = 自動割当）
  uint32 priority      = 3;  // 0 = 最高, 7 = 最低（CAN アービトレーション優先度）
  uint32 period_ms     = 4;  // 周期送信の場合のインターバル（0 = イベント駆動）
  string ros2_topic    = 5;  // ROS 2 トピック名オーバーライド（省略時は自動）
}

message ParamOptions {
  uint32 param_id    = 1;  // パラメータ番号（ファイル内で一意）
  bool   read_only   = 2;
  string ros2_param  = 3;  // ROS 2 パラメータ名オーバーライド
}

extend google.protobuf.MessageOptions {
  TopicOptions topic = 50001;
  ParamOptions param = 50002;
}
```

### 2.2 デバイススキーマ例: モータードライバ

```protobuf
// motor_driver.proto
syntax = "proto3";
package motor_driver;

import "protocan_options.proto";

option (protocan.device) = {
  device_type_id: 0x0010,
  device_type_name: "BLDCMotorDriver",
  schema_version: 1,
  ros2_namespace: "drive"
};

// ──────────── トピック定義 ────────────

// デバイスが 100Hz で送信するステータス
message MotorStatus {
  option (protocan.topic) = {
    direction: PUBLISH,
    channel_id: 1,
    priority: 3,
    period_ms: 10
  };

  float  current_a     = 1;  // [A] 相電流
  float  velocity_rps  = 2;  // [rev/s] 回転速度
  float  temperature_c = 3;  // [°C] 温度
  uint32 error_flags   = 4;  // ビットフィールド
}

// デバイスが 1kHz で送信するエンコーダ値
message EncoderFeedback {
  option (protocan.topic) = {
    direction: PUBLISH,
    channel_id: 2,
    priority: 1,
    period_ms: 1
  };

  int32 position_cnt = 1;  // エンコーダカウント
  int32 velocity_cps = 2;  // カウント/秒
}

// PC またはデバイスから受信するコマンド
message MotorCommand {
  option (protocan.topic) = {
    direction: SUBSCRIBE,
    channel_id: 3,
    priority: 2,
    period_ms: 0  // イベント駆動
  };

  float target_velocity_rps = 1;
  float torque_limit_a      = 2;
  uint32 control_mode       = 3;  // 0=velocity, 1=torque, 2=position
}

// ──────────── パラメータ定義 ────────────

message PidGains {
  option (protocan.param) = {
    param_id: 1,
    read_only: false,
    ros2_param: "pid_gains"
  };

  float kp = 1;
  float ki = 2;
  float kd = 3;
}

message DeviceInfo {
  option (protocan.param) = {
    param_id: 2,
    read_only: true,
    ros2_param: "device_info"
  };

  string firmware_version = 1;
  uint32 serial_number    = 2;
}
```

### 2.3 型マッピング（Proto型 → バイナリ表現）

データフレーム上では Protobuf のワイヤフォーマットを使わず、
**フィールド番号順に固定長で詰める（packed layout）** ことでオーバーヘッドを排除する。

| Proto 型      | CAN バイナリ  | サイズ   |
|--------------|-------------|---------|
| `bool`       | `uint8`     | 1 byte  |
| `uint32`     | little-endian u32 | 4 bytes |
| `int32`      | little-endian i32 | 4 bytes |
| `float`      | IEEE 754 LE | 4 bytes |
| `double`     | IEEE 754 LE | 8 bytes |
| `uint64`     | little-endian u64 | 8 bytes |
| `int64`      | little-endian i64 | 8 bytes |
| `bytes`      | (バルク転送のみ) | 可変長 |
| `string`     | (バルク転送のみ) | 可変長 |

> **注意**: 1 フレーム（最大 64 bytes）に収まるメッセージのみ DATA チャネルで送信可能。
> 収まらない場合はバルク転送プロトコルを使用する。

例: `MotorStatus` のフレームペイロード（16 bytes）:

```
Offset  Size  Field
0x00    4     current_a      (float LE)
0x04    4     velocity_rps   (float LE)
0x08    4     temperature_c  (float LE)
0x0C    4     error_flags    (uint32 LE)
```

---

## 3. CAN FD フレーム構造

### 3.1 CAN ID レイアウト（29-bit 拡張 ID）

CAN のアービトレーションでは ID の MSB から比較されるため、
**最上位ビットに優先度を配置** する。

```
Bit  [28:26]  Priority       (3 bits)  0=最高, 7=最低
Bit  [25:22]  Function Code  (4 bits)  メッセージ種別
Bit  [21:15]  Node ID        (7 bits)  送信元ノード (0–127)
Bit  [14:0]   Channel        (15 bits) トピック/パラメータ/コンテキスト依存
```

合計: 3 + 4 + 7 + 15 = 29 bits

### 3.2 Function Code 一覧

| Code | 名称     | 方向       | 用途                              |
|------|---------|-----------|----------------------------------|
| 0x0  | SYNC    | マスタ→全体 | 同期信号（タイムスタンプ配布）          |
| 0x1  | EMCY    | デバイス→全体| 緊急通知（障害・異常）                |
| 0x2  | NMT     | 双方向     | ハートビート・状態管理                 |
| 0x3  | DISC    | 双方向     | ディスカバリプロトコル                 |
| 0x4  | DATA_H  | 双方向     | 高優先度データチャネル                 |
| 0x5  | DATA_M  | 双方向     | 中優先度データチャネル                 |
| 0x6  | DATA_L  | 双方向     | 低優先度データチャネル                 |
| 0x7  | DATA_B  | 双方向     | バックグラウンドデータチャネル           |
| 0x8  | PARAM   | 双方向     | パラメータ読み書き                    |
| 0x9  | BULK    | 双方向     | バルク転送（ディスクリプタ・ファームウェア）|
| 0xA–0xF | (予約) |          | 将来拡張用                          |

**データチャネルの優先度マッピング**:
スキーマの `priority` 値を Function Code にマッピングする:

| Schema priority | Function Code |
|----------------|---------------|
| 0–1            | DATA_H (0x4)  |
| 2–3            | DATA_M (0x5)  |
| 4–5            | DATA_L (0x6)  |
| 6–7            | DATA_B (0x7)  |

### 3.3 Channel フィールドの使い方

| Function Code | Channel (15 bits) の意味                           |
|--------------|---------------------------------------------------|
| SYNC         | 0 固定                                             |
| EMCY         | エラーコード                                         |
| NMT          | サブコマンド (BOOT, HEARTBEAT, RESET, ...)           |
| DISC         | サブプロトコル番号                                     |
| DATA_*       | `(device_type_id[7:0] << 7) | channel_id[6:0]`    |
| PARAM        | `(device_type_id[7:0] << 7) | param_id[6:0]`      |
| BULK         | transfer_id                                        |

> DATA の Channel 構成により、**同じ channel_id でもデバイス種別が異なれば別チャネルになる**。
> これにより、異種デバイスが同一バス上で channel_id を独立管理できる。

### 3.4 CAN ID の具体例

```
EncoderFeedback（モータードライバ Node 0x10, channel_id=2, priority=1）:

Priority = 0b001 (priority 1 → DATA_H)
FuncCode = 0b0100 (DATA_H)
Node ID  = 0b0010000 (0x10)
Channel  = (0x10 << 7) | 2 = 0x0802

CAN ID = 001_0100_0010000_000100000000010
       = 0x0A100802
       → 29-bit: 0x0A100802 (実際は 29bit に収まるよう計算)
```

※ 実装時は各フィールドをビットシフトで組み立てる:

```c
#define PROTOCAN_ID(pri, func, node, ch) \
    (((pri) << 26) | ((func) << 22) | ((node) << 15) | (ch))
```

---

## 4. プロトコル

### 4.1 ノード管理 (NMT)

#### ハートビート

各デバイスは定期的に（デフォルト 500ms）ハートビートを送信する。

```
CAN ID: PROTOCAN_ID(7, NMT, node_id, 0x01)  // 最低優先度
Payload (8 bytes):
  [0]    state        : uint8  (0=BOOT, 1=PREOP, 2=OPERATIONAL, 3=STOPPED)
  [1:2]  device_type  : uint16 LE (スキーマの device_type_id)
  [3]    schema_ver   : uint8  (スキーマバージョン)
  [4:7]  uptime_ms    : uint32 LE
```

#### 状態遷移

```
  BOOT ──(初期化完了)──► PREOP ──(START受信)──► OPERATIONAL
                           ▲                       │
                           └──(STOP受信)────────────┘
```

- **BOOT**: 起動直後。ディスクリプタ要求に応答可能。
- **PREOP**: 設定変更可能。DATA チャネルは停止。
- **OPERATIONAL**: 全チャネル稼働中。

### 4.2 ディスカバリプロトコル

PC がデバイスを検出し、そのスキーマ情報を取得するフロー:

```
┌──────┐                        ┌─────────┐
│  PC  │                        │ Device  │
└──┬───┘                        └────┬────┘
   │                                 │
   │     ◄── HEARTBEAT (NMT) ───────│  (デバイス起動・周期送信)
   │                                 │
   │  ※ 未知の device_type_id を検出   │
   │                                 │
   │── DISC_REQUEST (DISC) ────────► │  (ディスクリプタ要求)
   │                                 │
   │     ◄── DISC_RESPONSE ─────────│  (ディスクリプタ長を返答)
   │                                 │
   │── BULK_START ─────────────────► │  (バルク転送開始要求)
   │                                 │
   │     ◄── BULK_DATA [0] ────────│─┐
   │     ◄── BULK_DATA [1] ────────│ │ ディスクリプタ本体
   │     ◄── BULK_DATA [n] ────────│ │ (複数フレーム)
   │     ◄── BULK_END ─────────────│─┘
   │                                 │
   │  ※ ROS 2 トピック/パラメータ生成   │
   │                                 │
   │── NMT_START (NMT) ───────────► │  (OPERATIONAL へ遷移指示)
   │                                 │
```

#### ディスカバリ要求/応答

```
DISC_REQUEST:
  CAN ID: PROTOCAN_ID(6, DISC, pc_node_id, 0x0001)
  Payload:
    [0:1] target_device_type : uint16 LE (0xFFFF = 全デバイス)
    [2]   target_node_id     : uint8 (0xFF = ブロードキャスト)

DISC_RESPONSE:
  CAN ID: PROTOCAN_ID(6, DISC, device_node_id, 0x0002)
  Payload:
    [0:1] device_type  : uint16 LE
    [2:3] desc_length  : uint16 LE (ディスクリプタのバイト数)
    [4]   desc_crc8    : uint8 (整合性チェック)
```

### 4.3 デバイスディスクリプタ（バイナリ形式）

各デバイスの ROM/Flash に格納される、コンパクトなスキーマ記述。
コード生成時に `.proto` から自動生成される。

```
Descriptor Binary Format:
┌─────────────────────────────────────────┐
│ Header (8 bytes)                        │
│   [0:1]  magic          = 0x50CA ("PC") │
│   [2:3]  device_type_id : uint16 LE     │
│   [4]    schema_version : uint8         │
│   [5]    num_topics     : uint8         │
│   [6]    num_params     : uint8         │
│   [7]    name_length    : uint8         │
├─────────────────────────────────────────┤
│ Device Name (name_length bytes, UTF-8)  │
├─────────────────────────────────────────┤
│ Topic Descriptors (num_topics × 可変長)  │
│   Per topic:                            │
│     [0]    channel_id    : uint8        │
│     [1]    direction     : uint8 (0/1)  │
│     [2]    priority      : uint8        │
│     [3:4]  period_ms     : uint16 LE    │
│     [5]    num_fields    : uint8        │
│     [6]    name_length   : uint8        │
│     [7..]  name          : UTF-8        │
│     Per field:                          │
│       [0]  field_type    : uint8        │
│       [1]  name_length   : uint8        │
│       [2..] name         : UTF-8        │
├─────────────────────────────────────────┤
│ Param Descriptors (num_params × 可変長)  │
│   (Topic Descriptor と同様の構造)        │
│   追加: [+0] read_only : uint8          │
└─────────────────────────────────────────┘

Field Type Enum:
  0x01 = bool
  0x02 = uint8
  0x03 = int8
  0x04 = uint16
  0x05 = int16
  0x06 = uint32
  0x07 = int32
  0x08 = uint64
  0x09 = int64
  0x0A = float32
  0x0B = float64
```

> 典型的なデバイス（3 トピック、2 パラメータ）のディスクリプタサイズ: 約 100–200 bytes。
> CAN FD の 64 byte フレーム 3–4 個で転送可能。

### 4.4 データ転送

**ゼロオーバーヘッド**: データフレームのペイロードは純粋なフィールドデータのみ。
メッセージの構造は CAN ID（device_type + channel_id）から一意に決定される。

```
DATA フレーム:
  CAN ID: PROTOCAN_ID(mapped_priority, DATA_x, source_node, channel)
  Payload: フィールドを番号順に packed layout で配置
  DLC: メッセージサイズ（スキーマから静的に決定）
```

#### デバイス間通信

CAN はブロードキャストバスなので、あるデバイスが PUBLISH したデータは
**他のデバイスが直接受信できる**。

例: デバイス A が `MotorStatus` を PUBLISH → デバイス B がその CAN ID をフィルタ設定して受信

サブスクライブ側の CAN フィルタ設定はスキーマから静的に生成:

```c
// デバイス B のファームウェア（自動生成）
// デバイス A の MotorStatus を受信するフィルタ
protocan_subscribe(
    PROTOCAN_ID(/*any_pri*/0, DATA_H, node_a, motor_status_channel),
    PROTOCAN_MASK_FUNC_NODE_CH,  // priority 以外をマスク
    on_motor_status_received
);
```

### 4.5 パラメータアクセス

SDO（Service Data Object）ライクなリクエスト/レスポンスプロトコル。

```
PARAM_READ_REQ:
  CAN ID: PROTOCAN_ID(6, PARAM, requester_node, channel)
  Payload:
    [0]    command    = 0x01 (READ)
    [1]    target_node : uint8
    [2:3]  param_id    : uint16 LE

PARAM_READ_RES:
  CAN ID: PROTOCAN_ID(6, PARAM, responder_node, channel)
  Payload:
    [0]    command    = 0x81 (READ_RESPONSE)
    [1]    status     : uint8 (0=OK, 1=NOT_FOUND, 2=ERROR)
    [2..]  value      : packed binary (パラメータの全フィールド)

PARAM_WRITE_REQ:
  CAN ID: PROTOCAN_ID(6, PARAM, requester_node, channel)
  Payload:
    [0]    command    = 0x02 (WRITE)
    [1]    target_node : uint8
    [2:3]  param_id    : uint16 LE
    [4..]  value       : packed binary

PARAM_WRITE_RES:
  CAN ID: PROTOCAN_ID(6, PARAM, responder_node, channel)
  Payload:
    [0]    command    = 0x82 (WRITE_RESPONSE)
    [1]    status     : uint8
```

> 64 bytes に収まらないパラメータはバルク転送にフォールバックする。

### 4.6 バルク転送

64 bytes を超えるデータ（ディスクリプタ、ファームウェアアップデートなど）の転送。
ISO-TP (ISO 15765-2) に準拠した分割転送:

```
First Frame:
  [0:1]  total_length : uint16 LE
  [2..]  data[0..61]

Consecutive Frame:
  [0]    sequence     : uint8 (1–255, 0 でラップ)
  [1..]  data

Flow Control (受信側 → 送信側):
  [0]    fc_flag      : uint8 (0=Continue, 1=Wait, 2=Abort)
  [1]    block_size   : uint8 (0=制限なし)
  [2:3]  st_min_us    : uint16 LE (最小分離時間 μs)
```

---

## 5. ROS 2 ブリッジノード

### 5.1 自動生成される構成

スキーマとディスカバリ情報から、以下が自動的に作成される:

```
/protocan/
  ├── drive/                          # ros2_namespace
  │   ├── motor_driver_10/            # {device_type_name}_{node_id_hex}
  │   │   ├── motor_status            # Topic (Sub) ← デバイスの PUBLISH
  │   │   ├── encoder_feedback        # Topic (Sub) ← デバイスの PUBLISH
  │   │   ├── motor_command           # Topic (Pub) → デバイスの SUBSCRIBE
  │   │   └── Parameters:
  │   │       ├── pid_gains           # ROS 2 Parameter (R/W)
  │   │       └── device_info         # ROS 2 Parameter (R)
  │   └── motor_driver_11/            # 同じ種別の別ノード
  │       └── ...
  └── protocan_bridge/
      └── Parameters:
          ├── can_interface            # "can0"
          ├── heartbeat_timeout_ms     # 2000
          └── auto_start               # true
```

### 5.2 ブリッジノードの動作

```python
# 概念的な疑似コード
class ProtoCANBridge(Node):

    def __init__(self):
        # CAN FD ソケットを開く
        self.can = open_canfd("can0")

        # 既知デバイスのレジストリ（スキーマ情報含む）
        self.known_schemas = load_compiled_schemas()  # コード生成済みスキーマ
        self.active_devices = {}

        # NMT ハートビートの受信開始
        self.can.set_filter(func=NMT)
        self.create_timer(0.1, self.poll_can)

    def on_heartbeat(self, node_id, device_type_id, schema_ver):
        if node_id not in self.active_devices:
            if device_type_id in self.known_schemas:
                # コンパイル済みスキーマから即座にトピック生成
                self.setup_device(node_id, self.known_schemas[device_type_id])
            else:
                # 未知のデバイス → ディスクリプタを要求
                self.request_descriptor(node_id, device_type_id)

    def setup_device(self, node_id, schema):
        """ROS 2 トピック・パラメータを動的に生成"""
        ns = f"{schema.ros2_namespace}/{schema.device_type_name}_{node_id:02x}"
        for topic in schema.topics:
            if topic.direction == PUBLISH:
                # デバイスの PUBLISH → ROS 2 Publisher を作成
                pub = self.create_publisher(topic.ros2_msg_type, f"{ns}/{topic.name}")
                self.can.set_filter(topic.can_id, callback=lambda d: pub.publish(decode(d)))
            else:
                # デバイスの SUBSCRIBE → ROS 2 Subscriber を作成
                sub = self.create_subscription(
                    topic.ros2_msg_type, f"{ns}/{topic.name}",
                    lambda msg: self.can.send(topic.can_id, encode(msg))
                )
```

### 5.3 ROS 2 メッセージ型の生成

コード生成時に、各トピック用の `.msg` ファイルも生成する:

```
# auto-generated: motor_driver/msg/MotorStatus.msg
float32 current_a
float32 velocity_rps
float32 temperature_c
uint32  error_flags
```

---

## 6. コード生成パイプライン

### 6.1 ツールチェーン

```
motor_driver.proto
       │
       ▼
  ┌─────────────────┐
  │  protoc          │
  │  --plugin=       │
  │   protocan-gen   │
  └──┬──────────┬───┘
     │          │
     ▼          ▼
 firmware/   ros2_ws/
```

### 6.2 ファームウェア用 C ライブラリ出力

```
firmware/
  motor_driver/
    ├── protocan_motor_driver.h      # メッセージ構造体・定数
    ├── protocan_motor_driver.c      # エンコーダ/デコーダ
    ├── protocan_motor_driver_desc.c # ディスクリプタ (const ROM配置)
    └── protocan_motor_driver_desc.h
```

生成される C コードの例:

```c
// protocan_motor_driver.h (自動生成)
#pragma once
#include "protocan.h"
#include <stdint.h>

// ──── デバイス定数 ────
#define MOTOR_DRIVER_DEVICE_TYPE_ID  0x0010
#define MOTOR_DRIVER_SCHEMA_VERSION  1

// ──── メッセージ構造体 ────
typedef struct __attribute__((packed)) {
    float    current_a;
    float    velocity_rps;
    float    temperature_c;
    uint32_t error_flags;
} protocan_motor_status_t;
_Static_assert(sizeof(protocan_motor_status_t) == 16, "size mismatch");

typedef struct __attribute__((packed)) {
    int32_t position_cnt;
    int32_t velocity_cps;
} protocan_encoder_feedback_t;
_Static_assert(sizeof(protocan_encoder_feedback_t) == 8, "size mismatch");

typedef struct __attribute__((packed)) {
    float    target_velocity_rps;
    float    torque_limit_a;
    uint32_t control_mode;
} protocan_motor_command_t;
_Static_assert(sizeof(protocan_motor_command_t) == 12, "size mismatch");

// ──── パラメータ構造体 ────
typedef struct __attribute__((packed)) {
    float kp;
    float ki;
    float kd;
} protocan_pid_gains_t;

// ──── CAN ID ヘルパー ────
// node_id はランタイムで設定
#define MOTOR_STATUS_CAN_ID(node_id) \
    PROTOCAN_ID(3, PROTOCAN_FUNC_DATA_M, (node_id), \
                (MOTOR_DRIVER_DEVICE_TYPE_ID << 7) | 1)

#define ENCODER_FB_CAN_ID(node_id) \
    PROTOCAN_ID(1, PROTOCAN_FUNC_DATA_H, (node_id), \
                (MOTOR_DRIVER_DEVICE_TYPE_ID << 7) | 2)

#define MOTOR_CMD_CAN_ID(node_id) \
    PROTOCAN_ID(2, PROTOCAN_FUNC_DATA_M, (node_id), \
                (MOTOR_DRIVER_DEVICE_TYPE_ID << 7) | 3)

// ──── API ────
void protocan_motor_driver_init(protocan_node_t *node, uint8_t node_id);

// Publish (送信)
protocan_status_t protocan_publish_motor_status(
    protocan_node_t *node,
    const protocan_motor_status_t *msg
);

protocan_status_t protocan_publish_encoder_feedback(
    protocan_node_t *node,
    const protocan_encoder_feedback_t *msg
);

// Subscribe コールバック登録
void protocan_on_motor_command(
    protocan_node_t *node,
    void (*callback)(const protocan_motor_command_t *msg, void *ctx),
    void *ctx
);
```

ファームウェアでの使用例:

```c
// main.c
#include "protocan_motor_driver.h"

protocan_node_t node;

void motor_command_handler(const protocan_motor_command_t *cmd, void *ctx) {
    set_motor_target(cmd->target_velocity_rps, cmd->torque_limit_a);
}

int main(void) {
    canfd_init();
    protocan_motor_driver_init(&node, 0x10);  // Node ID = 0x10
    protocan_on_motor_command(&node, motor_command_handler, NULL);
    protocan_start(&node);  // ハートビート開始、PREOP 状態

    while (1) {
        protocan_poll(&node);  // 受信処理・ハートビート送信

        // 周期的にステータスを送信
        protocan_motor_status_t status = {
            .current_a    = read_current(),
            .velocity_rps = read_velocity(),
            .temperature_c = read_temperature(),
            .error_flags  = get_errors(),
        };
        protocan_publish_motor_status(&node, &status);
    }
}
```

### 6.3 ROS 2 パッケージ出力

```
ros2_ws/src/
  protocan_msgs/
    ├── msg/
    │   ├── MotorStatus.msg
    │   ├── EncoderFeedback.msg
    │   └── MotorCommand.msg
    └── CMakeLists.txt

  protocan_bridge/
    ├── src/
    │   ├── bridge_node.cpp           # メインブリッジ
    │   ├── motor_driver_codec.cpp    # エンコード/デコード（自動生成）
    │   └── motor_driver_codec.h
    ├── config/
    │   └── default.yaml
    └── CMakeLists.txt
```

---

## 7. ノード ID の管理

### 7.1 割当方式

以下の 3 方式から選択可能（ブリッジの設定で切替）:

| 方式 | 説明 | ユースケース |
|------|------|-------------|
| **固定割当** | デバイスの DIP スイッチ / Flash で設定 | 本番環境 |
| **自動割当** | PC がディスカバリ時に未使用 ID を付与 | プロトタイピング |
| **ハッシュ割当** | デバイスシリアル番号の下位 7 bit | 中規模システム |

### 7.2 アドレス空間

```
Node ID 0x00       : ブロードキャスト（送信先として使用）
Node ID 0x01       : PC (ROS 2 ブリッジ) ※デフォルト
Node ID 0x02–0x0F  : 予約（将来の PC / ゲートウェイ用）
Node ID 0x10–0x7E  : 一般デバイス (111 台)
Node ID 0x7F       : 未割当デバイス（自動割当待ち）
```

---

## 8. エラーハンドリングと堅牢性

### 8.1 ハートビートタイムアウト

- PC 側: デバイスのハートビートが途絶えたら ROS 2 トピックに `stale` マーカーを発行
- デバイス側: PC からの START が一定時間来ない場合は自律動作モードへ

### 8.2 スキーマバージョン不一致

ハートビートに含まれる `schema_version` がコンパイル済みスキーマと異なる場合:

1. ディスクリプタを再取得
2. ランタイムでデコーダを動的構築（ディスクリプタに十分な型情報がある）
3. ログ警告を発行

### 8.3 バスオフ回復

CAN コントローラの Bus-Off 状態からの自動回復:

- 指数バックオフで再参加
- 回復後にハートビートで再アナウンス

---

## 9. 今後の検討事項

- [ ] タイムスタンプ同期 (SYNC フレームの詳細設計)
- [ ] ファームウェアアップデート over CAN FD (BULK 転送活用)
- [ ] CAN FD ↔ Ethernet ゲートウェイ対応
- [ ] セキュリティ (CAN 認証、SecOC 連携)
- [ ] `protoc` プラグインの具体的実装設計
- [ ] 可変長フィールド（`repeated`, `oneof`）の CAN 上での扱い
- [ ] QoS / デッドライン監視の ROS 2 側マッピング
- [ ] デバイス間通信の購読管理（静的 vs 動的フィルタ）
