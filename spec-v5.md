# ProtoCAN — CAN FD プラグ・アンド・プレイ通信システム仕様 (Draft v0.5)

## 1. 概要

ProtoCAN は、CAN FD バス上で複数の組み込みデバイスと PC（ROS 2）が双方向通信を行うためのプロトコルである。

### v0.4 からの主な変更

- **`service` / `rpc` 構文によるスキーマ定義**: `Direction` enum を廃止し、
  `stream` キーワードの位置で方向を推論。Topic と Param の区別も暗黙的に決定
- **カスタムオプションの大幅削減**: `service` = Node Type、`rpc` = Topic or Param
- **`service` 名がそのまま Node Type 名に**: 明示的な `node_type_name` を省略可能

### 設計思想

```
gRPC の service/rpc 構文を借用し、CAN FD のデータフローを宣言的に表現する。

  rpc の stream キーワード  →  CAN 上のデータ方向
  rpc の request/response  →  パラメータの読み書き
  service                  →  Node Type
```

---

## 2. スキーマ定義 (.proto)

### 2.1 カスタムオプション定義

```protobuf
// protocan/options.proto
syntax = "proto3";
package protocan;

import "google/protobuf/descriptor.proto";

// ══════════════════════════════════════════════════
//  Service レベル = Node Type
// ══════════════════════════════════════════════════

message NodeOptions {
  string ros2_namespace = 1;  // 省略時: service 名を snake_case 化
}

extend google.protobuf.ServiceOptions {
  NodeOptions node = 50000;
}

// ══════════════════════════════════════════════════
//  Method レベル = Topic or Param (stream の有無で自動判別)
// ══════════════════════════════════════════════════

message MethodOptions {
  // Topic 用
  bool   periodic       = 1;  // true: 周期送信（周期値はランタイムパラメータ）
  uint32 priority       = 2;  // 0=最高, 7=最低

  // Param 用
  bool   read_only      = 3;

  // ROS 2 マッピング
  string ros2_name      = 4;  // トピック名 or パラメータ名のオーバーライド
  string ros2_msg_type  = 5;  // 既存 ROS 2 型 (例: "sensor_msgs/msg/Imu")
}

extend google.protobuf.MethodOptions {
  MethodOptions method = 50001;
}

// ══════════════════════════════════════════════════
//  Field レベル = ROS 2 フィールドマッピング
// ══════════════════════════════════════════════════

message FieldOptions {
  string ros2_field = 1;  // "linear_acceleration.x" 等
}

extend google.protobuf.FieldOptions {
  FieldOptions field = 50003;
}
```

### 2.2 RPC パターンとセマンティクスの対応

`stream` キーワードの位置で、Topic/Param と方向を自動判別する:

```
┌──────────────────────────────────────────────────────────────────────────┐
│  rpc パターン                       │ 意味             │ CAN 上の動作    │
├──────────────────────────────────────────────────────────────────────────┤
│  rpc X(..) returns (stream Msg)     │ Device → Bus     │ TX Topic       │
│  (server streaming)                 │ デバイスが publish │ PDO 送信       │
├──────────────────────────────────────────────────────────────────────────┤
│  rpc X(stream Msg) returns (..)     │ Bus → Device     │ RX Topic       │
│  (client streaming)                 │ デバイスが subscribe│ PDO 受信      │
├──────────────────────────────────────────────────────────────────────────┤
│  rpc X(Msg) returns (Msg)           │ Read/Write Param │ PARAM R/W      │
│  (unary, same type)                 │ 双方向パラメータ   │ SDO ライク     │
├──────────────────────────────────────────────────────────────────────────┤
│  rpc X(Empty) returns (Msg)         │ Read-only Param  │ PARAM R        │
│  (unary, request が Empty)          │ 読み取り専用       │                │
└──────────────────────────────────────────────────────────────────────────┘

※ Empty = google.protobuf.Empty (または protocan が提供する空メッセージ)
※ stream 側のメッセージ型がデータ本体、非 stream 側の Empty は省略的記法

判定ルール (protoc プラグイン):
  1. request または response に stream がある → Topic
     - response が stream → DEVICE_TO_BUS (TX)
     - request が stream  → BUS_TO_DEVICE (RX)
  2. stream がない → Param
     - request が Empty → read_only = true (自動推論)
     - request == response 型 → read_only = false (自動推論)
     - MethodOptions.read_only で明示オーバーライド可能
```

### 2.3 例: IMU (ROS 2 標準型を再利用)

```protobuf
// imu.proto
syntax = "proto3";
package imu;

import "google/protobuf/empty.proto";
import "protocan/options.proto";

// ──── データ型定義 ────

message ImuData {
  float accel_x = 1 [(protocan.field) = { ros2_field: "linear_acceleration.x" }];
  float accel_y = 2 [(protocan.field) = { ros2_field: "linear_acceleration.y" }];
  float accel_z = 3 [(protocan.field) = { ros2_field: "linear_acceleration.z" }];
  float gyro_x  = 4 [(protocan.field) = { ros2_field: "angular_velocity.x" }];
  float gyro_y  = 5 [(protocan.field) = { ros2_field: "angular_velocity.y" }];
  float gyro_z  = 6 [(protocan.field) = { ros2_field: "angular_velocity.z" }];
}

message ImuCalibration {
  float gyro_offset_x = 1;
  float gyro_offset_y = 2;
  float gyro_offset_z = 3;
}

message ImuInfo {
  uint32 firmware_version = 1;
  uint32 serial_number    = 2;
}

// ──── Node Type 定義 ────

service IMU {
  option (protocan.node) = { ros2_namespace: "imu" };

  // Device → Bus: IMU データを周期送信
  // returns (stream ...) = デバイスが publish
  rpc Imu(google.protobuf.Empty) returns (stream ImuData) {
    option (protocan.method) = {
      periodic: true,
      priority: 1,
      ros2_msg_type: "sensor_msgs/msg/Imu"
    };
  }

  // Read/Write パラメータ: キャリブレーション値
  // unary, request == response → 暗黙的に read_only: false
  rpc Calibration(ImuCalibration) returns (ImuCalibration) {}

  // Read-only パラメータ: デバイス情報
  // unary, request が Empty → 暗黙的に read_only: true
  rpc Info(google.protobuf.Empty) returns (ImuInfo) {}
}
```

### 2.4 例: BLDC モーター (カスタム型 + 既存型混在)

```protobuf
// bldc_motor.proto
syntax = "proto3";
package bldc_motor;

import "google/protobuf/empty.proto";
import "protocan/options.proto";

// ──── データ型定義 ────

message MotorStatus {
  float  current_a     = 1;
  float  velocity_rps  = 2;
  float  temperature_c = 3;
  uint32 error_flags   = 4;
}

message EncoderFeedback {
  int32 position_cnt = 1;
  int32 velocity_cps = 2;
}

message TwistCommand {
  float linear_x  = 1 [(protocan.field) = { ros2_field: "linear.x" }];
  float linear_y  = 2 [(protocan.field) = { ros2_field: "linear.y" }];
  float linear_z  = 3 [(protocan.field) = { ros2_field: "linear.z" }];
  float angular_x = 4 [(protocan.field) = { ros2_field: "angular.x" }];
  float angular_y = 5 [(protocan.field) = { ros2_field: "angular.y" }];
  float angular_z = 6 [(protocan.field) = { ros2_field: "angular.z" }];
}

message PidGains {
  float kp = 1;
  float ki = 2;
  float kd = 3;
}

message DeviceInfo {
  uint32 firmware_version = 1;
  uint32 serial_number    = 2;
}

// ──── Node Type 定義 ────

service BLDCMotor {
  option (protocan.node) = { ros2_namespace: "motor" };

  // ──── Topics (stream あり) ────

  // Device → Bus: ステータス周期送信
  rpc Status(google.protobuf.Empty) returns (stream MotorStatus) {
    option (protocan.method) = { periodic: true, priority: 3 };
  }

  // Device → Bus: エンコーダ周期送信
  rpc Encoder(google.protobuf.Empty) returns (stream EncoderFeedback) {
    option (protocan.method) = { periodic: true, priority: 1 };
  }

  // Bus → Device: 速度コマンド (geometry_msgs/Twist として ROS 2 に公開)
  rpc Command(stream TwistCommand) returns (google.protobuf.Empty) {
    option (protocan.method) = {
      priority: 2,
      ros2_msg_type: "geometry_msgs/msg/Twist",
      ros2_name: "cmd_vel"
    };
  }

  // ──── Params (stream なし) ────

  // Read/Write: PID ゲイン
  rpc Pid(PidGains) returns (PidGains) {}

  // Read-only: デバイス情報
  rpc Info(google.protobuf.Empty) returns (DeviceInfo) {}
}
```

### 2.5 v0.4 との比較

```
v0.4 (message + custom options):
┌─────────────────────────────────────────────────────┐
│ message MotorStatus {                               │
│   option (protocan.topic) = {                       │
│     direction: DEVICE_TO_BUS,  ← 明示的に指定       │
│     periodic: true,                                 │
│     priority: 3                                     │
│   };                                                │
│   float current_a = 1;                              │
│   ...                                               │
│ }                                                   │
└─────────────────────────────────────────────────────┘

v0.5 (service/rpc):
┌─────────────────────────────────────────────────────┐
│ rpc Status(..) returns (stream MotorStatus) {       │
│   option (protocan.method) = {                      │
│     periodic: true, priority: 3                     │
│   };                                                │
│ }                                                   │
│                                                     │
│ stream キーワードの位置で方向が決まる                  │
│ ├─ returns (stream X) → DEVICE_TO_BUS              │
│ └─ (stream X) returns → BUS_TO_DEVICE              │
│                                                     │
│ stream の有無で Topic/Param が決まる                  │
│ ├─ stream あり → Topic                              │
│ └─ stream なし → Param                              │
│                                                     │
│ Empty の位置で read_only が決まる                     │
│ ├─ rpc X(Empty) returns (Y) → read_only            │
│ └─ rpc X(Y) returns (Y)    → read/write            │
└─────────────────────────────────────────────────────┘
```

**削減されたもの**:

- `Direction` enum (廃止)
- `TopicOptions` / `ParamOptions` の分離 (統合)
- `node_type_name` (service 名から自動取得)
- `direction` フィールド (stream キーワードから推論)
- `read_only` の多くのケース (Empty パターンから推論)

### 2.6 型マッピング

（v0.4 から変更なし）

| Proto 型 | CAN バイナリ | サイズ | ROS 2 型 |
|----------|-------------|-------|---------|
| `bool`   | uint8 (0/1) | 1B | bool |
| `uint32` | LE u32 | 4B | uint32 |
| `int32`  | LE i32 | 4B | int32 |
| `float`  | IEEE 754 LE | 4B | float32 |
| `double` | IEEE 754 LE | 8B | float64 |
| `uint64` | LE u64 | 8B | uint64 |
| `int64`  | LE i64 | 8B | int64 |

---

## 3. スキーマハッシュ

### 3.1 ハッシュ入力

```
正規化手順:
1. service 内の rpc を定義順にソート
2. 各 rpc について:
   a. 種別: 'T' (Topic = stream あり) or 'P' (Param = stream なし)
   b. Topic の場合: direction (1B, stream 位置から), periodic (1B), priority (1B)
      Param の場合: read_only (1B)
   c. ros2_msg_type の UTF-8 (空可)
   d. データメッセージの構造:
      - num_fields (1B)
      - 各 field: field_number (varint), field_type (1B), ros2_field UTF-8 (空可)
3. 全 rpc を連結 → FNV-1a 32-bit
```

### 3.2 キャッシュフロー

（v0.4 から変更なし — ハートビートの schema_hash でキャッシュ照合）

---

## 4. コード生成 (ファームウェア側のみ)

### 4.1 protoc プラグインの処理

```
入力: bldc_motor.proto (service BLDCMotor)
         │
         ▼
  protoc-gen-protocan が解析:
    service BLDCMotor → Node Type 名 = "BLDCMotor"
    rpc Status(..) returns (stream MotorStatus)
      → Topic, DEVICE_TO_BUS, periodic=true, priority=3
      → MotorStatus のフィールド → packed struct 生成
    rpc Encoder(..) returns (stream EncoderFeedback)
      → Topic, DEVICE_TO_BUS, periodic=true, priority=1
    rpc Command(stream TwistCommand) returns (..)
      → Topic, BUS_TO_DEVICE, periodic=false, priority=2
      → ros2_msg_type="geometry_msgs/msg/Twist"
    rpc Pid(PidGains) returns (PidGains)
      → Param, read_only=false
    rpc Info(Empty) returns (DeviceInfo)
      → Param, read_only=true (Empty パターンから推論)
         │
         ▼
  出力:
    protocan_bldc_motor.h      — 構造体・API
    protocan_bldc_motor.c      — 実装
    protocan_bldc_motor_desc.c — ディスクリプタ片 (ROM, ROS 2 情報含む)
```

### 4.2 生成コード例

```c
// ============================================================
// protocan_bldc_motor.h (自動生成 from service BLDCMotor)
// ============================================================
#pragma once
#include "protocan/protocan.h"

#define BLDC_MOTOR_SCHEMA_HASH  0xA3F7B201u

// ──── Topic/Param Index (rpc 定義順) ────
// Topics:
#define BLDC_MOTOR_TOPIC_STATUS    0  // rpc Status
#define BLDC_MOTOR_TOPIC_ENCODER   1  // rpc Encoder
#define BLDC_MOTOR_TOPIC_COMMAND   2  // rpc Command
// Params:
#define BLDC_MOTOR_PARAM_PID       0  // rpc Pid
#define BLDC_MOTOR_PARAM_INFO      1  // rpc Info

// ──── メッセージ構造体 ────
// rpc のデータメッセージ型から生成

typedef struct __attribute__((packed)) {
    float    current_a;
    float    velocity_rps;
    float    temperature_c;
    uint32_t error_flags;
} protocan_motor_status_t;  // 16B
_Static_assert(sizeof(protocan_motor_status_t) == 16, "");

typedef struct __attribute__((packed)) {
    int32_t position_cnt;
    int32_t velocity_cps;
} protocan_encoder_feedback_t;  // 8B

typedef struct __attribute__((packed)) {
    float linear_x, linear_y, linear_z;
    float angular_x, angular_y, angular_z;
} protocan_twist_command_t;  // 24B

typedef struct __attribute__((packed)) {
    float kp, ki, kd;
} protocan_pid_gains_t;  // 12B

typedef struct __attribute__((packed)) {
    uint32_t firmware_version;
    uint32_t serial_number;
} protocan_device_info_t;  // 8B

// ──── ノードハンドル ────

typedef struct {
    protocan_node_t base;

    // TX バッファ (DEVICE_TO_BUS topics)
    protocan_motor_status_t      status;
    protocan_encoder_feedback_t  encoder;

    // RX バッファ (BUS_TO_DEVICE topics)
    protocan_twist_command_t     command;

    // パラメータ
    protocan_pid_gains_t         pid;
    protocan_device_info_t       info;

    // RX コールバック
    void (*on_command)(const protocan_twist_command_t*, void*);
    void *on_command_ctx;
} protocan_bldc_motor_node_t;

// ──── API ────

void protocan_bldc_motor_init(
    protocan_bldc_motor_node_t *node,
    uint8_t instance_index,
    const char *instance_name  // "left", "right" 等
);

// Publish (DEVICE_TO_BUS)
protocan_status_t protocan_bldc_motor_publish_status(
    protocan_bldc_motor_node_t *node);
protocan_status_t protocan_bldc_motor_publish_encoder(
    protocan_bldc_motor_node_t *node);

// Subscribe callback (BUS_TO_DEVICE)
void protocan_bldc_motor_on_command(
    protocan_bldc_motor_node_t *node,
    void (*cb)(const protocan_twist_command_t*, void*),
    void *ctx);
```

### 4.3 ディスクリプタ片

```c
// protocan_bldc_motor_desc.c (自動生成)

static const protocan_field_desc_t status_fields[] = {
    { "current_a",     PROTOCAN_FLOAT32, 0,  4, NULL },
    { "velocity_rps",  PROTOCAN_FLOAT32, 4,  4, NULL },
    { "temperature_c", PROTOCAN_FLOAT32, 8,  4, NULL },
    { "error_flags",   PROTOCAN_UINT32,  12, 4, NULL },
};

static const protocan_field_desc_t command_fields[] = {
    { "linear_x",  PROTOCAN_FLOAT32, 0,  4, "linear.x" },
    { "linear_y",  PROTOCAN_FLOAT32, 4,  4, "linear.y" },
    { "linear_z",  PROTOCAN_FLOAT32, 8,  4, "linear.z" },
    { "angular_x", PROTOCAN_FLOAT32, 12, 4, "angular.x" },
    { "angular_y", PROTOCAN_FLOAT32, 16, 4, "angular.y" },
    { "angular_z", PROTOCAN_FLOAT32, 20, 4, "angular.z" },
};

// rpc 定義順にインデックス付与
static const protocan_topic_desc_t bldc_motor_topics[] = {
    { // rpc Status → topic_index = 0
        .name = "status",  // rpc 名を snake_case 化
        .direction  = PROTOCAN_DIR_DEVICE_TO_BUS,  // returns (stream ..)
        .periodic = true, .priority = 3, .total_size = 16,
        .ros2_msg_type = NULL, .ros2_name = NULL,
        .fields = status_fields, .num_fields = 4,
    },
    { // rpc Encoder → topic_index = 1
        .name = "encoder",
        .direction = PROTOCAN_DIR_DEVICE_TO_BUS,
        .periodic = true, .priority = 1, .total_size = 8,
        .ros2_msg_type = NULL, .ros2_name = NULL,
        .fields = encoder_fields, .num_fields = 2,
    },
    { // rpc Command → topic_index = 2
        .name = "cmd_vel",  // ros2_name オーバーライド
        .direction = PROTOCAN_DIR_BUS_TO_DEVICE,  // (stream ..) returns
        .periodic = false, .priority = 2, .total_size = 24,
        .ros2_msg_type = "geometry_msgs/msg/Twist",
        .ros2_name = "cmd_vel",
        .fields = command_fields, .num_fields = 6,
    },
};

static const protocan_param_desc_t bldc_motor_params[] = {
    { // rpc Pid → param_index = 0
        .name = "pid",
        .read_only = false,   // Pid(PidGains) returns (PidGains) → R/W
        .total_size = 12,
        .ros2_msg_type = NULL, .ros2_name = NULL,
        .fields = pid_fields, .num_fields = 3,
    },
    { // rpc Info → param_index = 1
        .name = "info",
        .read_only = true,    // Info(Empty) returns (DeviceInfo) → R/O
        .total_size = 8,
        .ros2_msg_type = NULL, .ros2_name = NULL,
        .fields = info_fields, .num_fields = 2,
    },
};

const protocan_node_type_desc_t PROTOCAN_BLDC_MOTOR_DESC = {
    .node_type_name = "BLDCMotor",  // service 名
    .schema_hash    = BLDC_MOTOR_SCHEMA_HASH,
    .ros2_namespace = "motor",
    .topics     = bldc_motor_topics,  .num_topics = 3,
    .params     = bldc_motor_params,  .num_params = 2,
};
```

### 4.4 ファームウェア使用例

```c
// main.c — DualMotorBoard
#include "protocan/protocan.h"
#include "generated/protocan_bldc_motor.h"
#include "generated/protocan_imu.h"

static protocan_device_t device;
static protocan_bldc_motor_node_t motor_left;
static protocan_bldc_motor_node_t motor_right;
static protocan_imu_node_t        imu_node;

void on_left_cmd(const protocan_twist_command_t *cmd, void *ctx) {
    set_wheel_velocity(LEFT, cmd->linear_x, cmd->angular_z);
}
void on_right_cmd(const protocan_twist_command_t *cmd, void *ctx) {
    set_wheel_velocity(RIGHT, cmd->linear_x, cmd->angular_z);
}

int main(void) {
    canfd_hal_init();

    protocan_device_init(&device, "DualMotorBoard");
    protocan_device_set_id(&device, read_dip_switch());

    // ノード初期化 — C で構成を宣言
    protocan_bldc_motor_init(&motor_left,  0, "left");
    protocan_bldc_motor_init(&motor_right, 1, "right");
    protocan_imu_init(&imu_node, 0, NULL);

    // デバイスにノード登録 → local_node_id 自動割当 & ディスクリプタ連結
    protocan_device_add_node(&device, &motor_left.base);   // local=0
    protocan_device_add_node(&device, &motor_right.base);  // local=1
    protocan_device_add_node(&device, &imu_node.base);     // local=2

    // コールバック
    protocan_bldc_motor_on_command(&motor_left,  on_left_cmd,  NULL);
    protocan_bldc_motor_on_command(&motor_right, on_right_cmd, NULL);

    protocan_device_start(&device);

    while (1) {
        protocan_device_poll(&device);

        if (protocan_device_state(&device) == PROTOCAN_STATE_OPERATIONAL) {
            motor_left.status.current_a    = read_current(LEFT);
            motor_left.status.velocity_rps = read_velocity(LEFT);
            motor_left.status.temperature_c = read_temp(LEFT);
            motor_left.status.error_flags  = get_errors(LEFT);

            motor_left.encoder.position_cnt = read_encoder(LEFT);
            motor_left.encoder.velocity_cps = read_encoder_vel(LEFT);

            // right, imu 同様...

            protocan_bldc_motor_publish_status(&motor_left);
            protocan_bldc_motor_publish_status(&motor_right);
            protocan_bldc_motor_publish_encoder(&motor_left);
            protocan_bldc_motor_publish_encoder(&motor_right);
            protocan_imu_publish_imu(&imu_node);
        }
    }
}
```

---

## 5. 以降のセクション

CAN フレーム設計、PDO マッピング、デバイス間ルーティング、NMT、ディスカバリ、
デバイスディスクリプタ形式、protocan_bridge の動作は **v0.4 から変更なし**。

ディスクリプタ内部の topic/param 記述も同じバイナリ形式を使用。
変わるのは、protoc プラグインが `service`/`rpc` から情報を抽出する方法のみ。

---

## 6. 設計判断のまとめ

### v0.5 の変更

| 項目 | v0.4 | v0.5 | 理由 |
|------|------|------|------|
| スキーマ構造 | message + custom options | **service/rpc** | protobuf の標準構文を活用 |
| Direction | `Direction` enum で明示指定 | **`stream` の位置で推論** | 宣言的・暗黙的 |
| Topic / Param 区別 | `topic` / `param` option | **stream の有無で推論** | オプション統合 |
| read_only | 明示指定 | **Empty パターンから推論** (オーバーライド可) | 慣用的 |
| Node Type 名 | `node_type_name` option | **service 名** | 冗長性排除 |
| Options | TopicOptions + ParamOptions | **統一 MethodOptions** | 簡素化 |

### 全バージョンからの継続事項

| 項目 | 採用版 | 内容 |
|------|-------|------|
| CAN フレーム二層構成 | v0.2 | Standard ID: PDO データ / Extended ID: 管理 |
| PDO マッピング | v0.2 | マスター管理、ノード横断パッキング |
| デバイス間ルーティング | v0.2 | TX/RX PDO ペアリング |
| schema_hash | v0.3 | FNV-1a 32-bit でキャッシュ |
| ROS 2 既存型再利用 | v0.3 | `ros2_msg_type` + `ros2_field` |
| C API デバイス構成 | v0.4 | `protocan_device_add_node()` |
| 汎用ブリッジ | v0.4 | コード生成不要、ディスクリプタ駆動 |
| ディスクリプタに ROS 情報 | v0.4 | ブリッジのスキーマ事前知識不要 |
| service/rpc 構文 | v0.5 | stream で方向推論 |

---

## 7. 今後の検討事項

- [ ] `protoc-gen-protocan` プラグイン実装 (ServiceDescriptorProto の解析)
- [ ] protocan_bridge 汎用 C++ 実装
- [ ] `rosidl_typesupport_introspection` ベースの DynamicMsgWriter
- [ ] bidirectional stream (`stream X returns stream Y`) の扱い — 将来の双方向ストリーミングトピック？
- [ ] ros2_msg_type 省略時の動的型方式の最終決定
- [ ] ディスクリプタ圧縮
- [ ] PDO マッピング最適化
- [ ] SYNC タイムスタンプ同期
- [ ] ファームウェアアップデート over CAN FD
- [ ] 複数マスター冗長構成
