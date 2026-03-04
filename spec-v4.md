# ProtoCAN — CAN FD プラグ・アンド・プレイ通信システム仕様 (Draft v0.4)

## 1. 概要

ProtoCAN は、CAN FD バス上で複数の組み込みデバイスと PC（ROS 2）が双方向通信を行うためのプロトコルである。

### v0.3 からの主な変更

- **device_manifest.toml を廃止**: デバイス構成は C/C++ ソースコード内で記述
- **protocan_bridge は汎用ランタイム**: コード生成不要。どんなデバイスにも動的に対応
- **ディスクリプタに ROS 2 情報を含める**: `ros2_msg_type`, `ros2_field` マッピングを
  デバイス ROM に格納。ブリッジはスキーマの事前知識なしに動作可能
- **ROS 2 メッセージの動的操作**: `rosidl_typesupport_introspection` を使用し、
  フィールド名ベースで実行時にメッセージを構築

### 設計思想

```
デバイスが自己記述する全情報（CAN レイアウト + ROS 2 マッピング）
 + 汎用ブリッジが動的にデコード & ROS 2 publish
 = コンパイル済みスキーマ一切不要のプラグ・アンド・プレイ
```

### 用語定義

| 用語 | 定義 |
|------|------|
| **Device** | 物理的な 1 デバイス（1 MCU / 1 CAN トランシーバ） |
| **Node** | Device 上の論理的な機能単位。1 `.proto` = 1 Node Type |
| **Node Instance** | 同一 Node Type の複数インスタンス |
| **Topic** | Node が送受信するデータストリーム |
| **Param** | Node のランタイム設定値 |
| **PDO** | Process Data Object — Standard CAN ID + フレームレイアウト |
| **Schema Hash** | `.proto` の構造から計算される 32-bit 識別子 |
| **Descriptor** | デバイス ROM に格納される自己記述データ（CAN + ROS 2 情報） |
| **Master** | PC 側の汎用 ProtoCAN ブリッジ |

### アーキテクチャ全体像

```
┌───────────────────────────────────────────────────────────────┐
│  .proto スキーマ群 (Node Type 定義)                              │
│  motor.proto, imu.proto, gpio.proto, ...                      │
└──────────┬────────────────────────────────────────────────────┘
           │  protoc-gen-protocan
           ▼
┌────────────────────────────────────────────────────────────┐
│  ファームウェア用 C ライブラリ (per Node Type)                   │
│  - packed 構造体 & エンコーダ/デコーダ                          │
│  - ディスクリプタ片 (const ROM) — ROS 2 情報を含む              │
│  - schema_hash 定数                                          │
└────────────┬───────────────────────────────────────────────┘
             │  ユーザーが C/C++ で組み立て
             ▼
┌────────────────────────────────────────────────────────────┐
│  デバイスファームウェア (main.c)                                │
│  - protocan_device_add_node() で構成を宣言                     │
│  - ディスクリプタはノード登録時にランタイムで連結                  │
└────────────┬───────────────────────────────────────────────┘
             │
             ▼
┌───────────────────────────────────────────────────────────────┐
│                        CAN FD バス                             │
│  [Standard ID: PDO データ]    [Extended ID: 管理]               │
└───────────────────────────────────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────────────────┐
│  protocan_bridge (汎用 ROS 2 ノード)                          │
│  - コード生成なし、コンパイル済みスキーマ不要                     │
│  - ディスクリプタから ROS 2 マッピングを動的構築                  │
│  - rosidl_typesupport_introspection でメッセージを動的操作      │
│  - schema_hash のキャッシュで 2 回目以降のディスクリプタ転送省略   │
└────────────────────────────────────────────────────────────┘
```

---

## 2. スキーマハッシュ

### 2.1 目的

- **高速一致判定**: ハートビートの数バイトでスキーマ照合
- **ディスクリプタキャッシュ**: ブリッジが一度受信したディスクリプタを hash → descriptor で
  キャッシュし、同じ hash のデバイスが再接続したら転送をスキップ

### 2.2 ハッシュ入力（正規化表現）

スキーマの**構造的内容と ROS 2 マッピング情報の両方**からハッシュを計算する。
（v0.4 ではディスクリプタに ROS 2 情報が含まれるため、マッピング変更もハッシュに反映）

```
正規化手順:
1. topic/param オプション付き message をファイル内出現順にソート
2. 各 message について以下をバイト列として連結:
   a. 種別: 'T' (topic) or 'P' (param)
   b. TopicOptions: direction (1B), periodic (1B), priority (1B)
      または ParamOptions: read_only (1B)
   c. ros2_msg_type の UTF-8 バイト列 (なければ空)
   d. num_fields (1B)
   e. 各 field (field_number 順):
      - field_number (varint)
      - field_type enum (1B)
      - ros2_field の UTF-8 バイト列 (なければ空)
3. 全 message を連結
4. FNV-1a 32-bit ハッシュ
```

### 2.3 ブリッジ側のキャッシュフロー

```
ハートビート受信 (schema_hash を含む)
            │
            ▼
  ブリッジのキャッシュに同じ hash がある？
        │                  │
       YES                 NO
        │                  │
        ▼                  ▼
  キャッシュ済み          ディスクリプタを
  ディスクリプタで        バルク転送で取得
  即セットアップ          → キャッシュに保存
```

> ブリッジにコンパイル済みスキーマは不要。
> 初回接続時にディスクリプタを受信してキャッシュするだけ。

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
  string node_type_name = 1;  // 人間可読名 (例: "BLDCMotor")
  string ros2_namespace = 2;  // ROS 2 名前空間プレフィックス（省略時は自動）
}

extend google.protobuf.FileOptions {
  NodeTypeOptions node_type = 50000;
}

// ══════════════════════════════════════════════════
//  メッセージレベル = Topic / Param
// ══════════════════════════════════════════════════

enum Direction {
  DEVICE_TO_BUS = 0;
  BUS_TO_DEVICE = 1;
}

message TopicOptions {
  Direction direction    = 1;
  bool     periodic      = 2;
  uint32   priority      = 3;  // 0=最高, 7=最低
  string   ros2_topic    = 4;  // ROS 2 トピック名オーバーライド
  string   ros2_msg_type = 5;  // 既存 ROS 2 型 (例: "sensor_msgs/msg/Imu")
                                // 省略時: ブリッジが動的メッセージ型を生成
}

message ParamOptions {
  bool   read_only      = 1;
  string ros2_param     = 2;
  string ros2_msg_type  = 3;
}

extend google.protobuf.MessageOptions {
  TopicOptions topic = 50001;
  ParamOptions param = 50002;
}

// ══════════════════════════════════════════════════
//  フィールドレベル
// ══════════════════════════════════════════════════

message FieldOptions {
  string ros2_field = 1;  // ROS 2 メッセージ内のフィールドパス
                           // (例: "linear_acceleration.x")
}

extend google.protobuf.FieldOptions {
  FieldOptions field = 50003;
}
```

### 3.2 例: IMU (ROS 2 標準型を再利用)

```protobuf
// imu.proto
syntax = "proto3";
package imu;

import "protocan/options.proto";

option (protocan.node_type) = {
  node_type_name: "IMU",
  ros2_namespace: "imu"
};

message ImuData {
  option (protocan.topic) = {
    direction: DEVICE_TO_BUS,
    periodic: true,
    priority: 1,
    ros2_msg_type: "sensor_msgs/msg/Imu"
  };

  float accel_x = 1 [(protocan.field) = { ros2_field: "linear_acceleration.x" }];
  float accel_y = 2 [(protocan.field) = { ros2_field: "linear_acceleration.y" }];
  float accel_z = 3 [(protocan.field) = { ros2_field: "linear_acceleration.z" }];
  float gyro_x  = 4 [(protocan.field) = { ros2_field: "angular_velocity.x" }];
  float gyro_y  = 5 [(protocan.field) = { ros2_field: "angular_velocity.y" }];
  float gyro_z  = 6 [(protocan.field) = { ros2_field: "angular_velocity.z" }];
}
```

### 3.3 例: BLDC モーター (カスタム型 + 既存型混在)

```protobuf
// bldc_motor.proto
syntax = "proto3";
package bldc_motor;

import "protocan/options.proto";

option (protocan.node_type) = {
  node_type_name: "BLDCMotor",
  ros2_namespace: "motor"
};

// ros2_msg_type 省略 → ブリッジが動的にメッセージ型を構築
message MotorStatus {
  option (protocan.topic) = {
    direction: DEVICE_TO_BUS,
    periodic: true,
    priority: 3
  };
  float  current_a     = 1;
  float  velocity_rps  = 2;
  float  temperature_c = 3;
  uint32 error_flags   = 4;
}

message EncoderFeedback {
  option (protocan.topic) = {
    direction: DEVICE_TO_BUS,
    periodic: true,
    priority: 1
  };
  int32 position_cnt = 1;
  int32 velocity_cps = 2;
}

// geometry_msgs/msg/Twist にマッピング
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

| `ros2_msg_type` | ブリッジの動作 |
|-----------------|-------------|
| **指定あり** | `rosidl_typesupport_introspection` で既存型をロードし、`ros2_field` パスで各フィールドにアクセス |
| **省略** | ディスクリプタのフィールド名・型から `protocan_msgs/msg/` 以下に動的メッセージ型を登録（または、`std_msgs` 相当の汎用ラッパー） |

### 3.5 型マッピング

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

## 4. コード生成 (ファームウェア側のみ)

### 4.1 生成対象と非生成対象

```
protoc-gen-protocan が生成するもの:
  ✅ ファームウェア用 C ライブラリ (per Node Type)
     - packed 構造体
     - schema_hash 定数
     - ノードディスクリプタ片 (const ROM) — ROS 2 情報含む
     - Publish / Subscribe API

protoc-gen-protocan が生成しないもの:
  ❌ device_manifest.toml (廃止)
  ❌ protocan_bridge のソースコード (汎用ランタイム)
  ❌ ROS 2 .msg ファイル (動的に処理)
  ❌ ROS 2 コンバータコード (動的に処理)
```

### 4.2 生成ファイル構成

```
firmware/
  protocan/                              # 共通ランタイム (手書き、全デバイス共通)
    ├── protocan.h / .c                  # コア API
    ├── protocan_device.h / .c           # デバイス管理・ノード登録
    ├── protocan_nmt.c                   # ハートビート・状態管理
    ├── protocan_pdo.c                   # PDO テーブル管理
    ├── protocan_param.c                 # パラメータアクセス
    ├── protocan_bulk.c                  # ISO-TP バルク転送
    ├── protocan_descriptor.h / .c       # ディスクリプタ連結・応答
    └── protocan_hal.h                   # HAL インターフェース (ユーザー実装)

  generated/                             # protoc プラグイン生成物 (per .proto)
    ├── protocan_bldc_motor.h / .c       # 構造体・API・schema_hash
    ├── protocan_bldc_motor_desc.c       # ノードディスクリプタ片 (const ROM)
    ├── protocan_imu.h / .c
    ├── protocan_imu_desc.c
    ├── protocan_gpio.h / .c
    └── protocan_gpio_desc.c
```

### 4.3 生成コード例: ヘッダ

```c
// ============================================================
// protocan_bldc_motor.h (自動生成)
// ============================================================
#pragma once
#include "protocan/protocan.h"

#define BLDC_MOTOR_SCHEMA_HASH  0xA3F7B201u

// Topic Index (ファイル内出現順で自動割当)
#define BLDC_MOTOR_TOPIC_MOTOR_STATUS      0
#define BLDC_MOTOR_TOPIC_ENCODER_FEEDBACK  1
#define BLDC_MOTOR_TOPIC_MOTOR_COMMAND     2

// Param Index
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
    float linear_x, linear_y, linear_z;
    float angular_x, angular_y, angular_z;
} protocan_motor_command_t;  // 24 bytes

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

// 初期化: ノードディスクリプタ片を base に登録
void protocan_bldc_motor_init(
    protocan_bldc_motor_node_t *node,
    uint8_t instance_index,    // 同型ノード内の番号 (0, 1, ...)
    const char *instance_name  // "left", "right" 等 (NULL 可)
);

protocan_status_t protocan_bldc_motor_publish_motor_status(
    protocan_bldc_motor_node_t *node);
protocan_status_t protocan_bldc_motor_publish_encoder_feedback(
    protocan_bldc_motor_node_t *node);

void protocan_bldc_motor_on_motor_command(
    protocan_bldc_motor_node_t *node,
    void (*cb)(const protocan_motor_command_t*, void*),
    void *ctx);
```

### 4.4 生成コード例: ノードディスクリプタ片

```c
// ============================================================
// protocan_bldc_motor_desc.c (自動生成)
// ============================================================
#include "protocan_bldc_motor.h"

// ROS 2 フィールドマッピング文字列 (NULL = proto フィールド名をそのまま使用)
static const protocan_field_desc_t motor_status_fields[] = {
    { .name = "current_a",     .type = PROTOCAN_FLOAT32, .offset = 0,  .size = 4, .ros2_field = NULL },
    { .name = "velocity_rps",  .type = PROTOCAN_FLOAT32, .offset = 4,  .size = 4, .ros2_field = NULL },
    { .name = "temperature_c", .type = PROTOCAN_FLOAT32, .offset = 8,  .size = 4, .ros2_field = NULL },
    { .name = "error_flags",   .type = PROTOCAN_UINT32,  .offset = 12, .size = 4, .ros2_field = NULL },
};

static const protocan_field_desc_t encoder_feedback_fields[] = {
    { .name = "position_cnt",  .type = PROTOCAN_INT32, .offset = 0, .size = 4, .ros2_field = NULL },
    { .name = "velocity_cps",  .type = PROTOCAN_INT32, .offset = 4, .size = 4, .ros2_field = NULL },
};

static const protocan_field_desc_t motor_command_fields[] = {
    { .name = "linear_x",  .type = PROTOCAN_FLOAT32, .offset = 0,  .size = 4, .ros2_field = "linear.x" },
    { .name = "linear_y",  .type = PROTOCAN_FLOAT32, .offset = 4,  .size = 4, .ros2_field = "linear.y" },
    { .name = "linear_z",  .type = PROTOCAN_FLOAT32, .offset = 8,  .size = 4, .ros2_field = "linear.z" },
    { .name = "angular_x", .type = PROTOCAN_FLOAT32, .offset = 12, .size = 4, .ros2_field = "angular.x" },
    { .name = "angular_y", .type = PROTOCAN_FLOAT32, .offset = 16, .size = 4, .ros2_field = "angular.y" },
    { .name = "angular_z", .type = PROTOCAN_FLOAT32, .offset = 20, .size = 4, .ros2_field = "angular.z" },
};

static const protocan_topic_desc_t bldc_motor_topics[] = {
    {
        .name       = "motor_status",
        .direction  = PROTOCAN_DIR_DEVICE_TO_BUS,
        .periodic   = true,
        .priority   = 3,
        .total_size = 16,
        .ros2_msg_type = NULL,    // カスタム型 → ブリッジが動的生成
        .ros2_topic    = NULL,    // デフォルト名
        .fields     = motor_status_fields,
        .num_fields = 4,
    },
    {
        .name       = "encoder_feedback",
        .direction  = PROTOCAN_DIR_DEVICE_TO_BUS,
        .periodic   = true,
        .priority   = 1,
        .total_size = 8,
        .ros2_msg_type = NULL,
        .ros2_topic    = NULL,
        .fields     = encoder_feedback_fields,
        .num_fields = 2,
    },
    {
        .name       = "motor_command",
        .direction  = PROTOCAN_DIR_BUS_TO_DEVICE,
        .periodic   = false,
        .priority   = 2,
        .total_size = 24,
        .ros2_msg_type = "geometry_msgs/msg/Twist",
        .ros2_topic    = NULL,
        .fields     = motor_command_fields,
        .num_fields = 6,
    },
};

// パラメータ (同様の構造)
static const protocan_field_desc_t pid_gains_fields[] = { /* ... */ };
static const protocan_param_desc_t bldc_motor_params[] = { /* ... */ };

// ──── ノードタイプディスクリプタ (ROM) ────

const protocan_node_type_desc_t PROTOCAN_BLDC_MOTOR_DESC = {
    .node_type_name = "BLDCMotor",
    .schema_hash    = BLDC_MOTOR_SCHEMA_HASH,
    .ros2_namespace = "motor",
    .topics         = bldc_motor_topics,
    .num_topics     = 3,
    .params         = bldc_motor_params,
    .num_params     = 2,
};
```

### 4.5 ファームウェア使用例 (manifest 不要)

```c
// ============================================================
// main.c — DualMotorBoard ファームウェア
// ============================================================
#include "protocan/protocan.h"
#include "protocan/protocan_device.h"
#include "generated/protocan_bldc_motor.h"
#include "generated/protocan_imu.h"

// ──── デバイス ────
static protocan_device_t device;

// ──── ノードインスタンス ────
static protocan_bldc_motor_node_t motor_left;
static protocan_bldc_motor_node_t motor_right;
static protocan_imu_node_t        imu_node;

// ──── コールバック ────
void on_left_cmd(const protocan_motor_command_t *cmd, void *ctx) {
    set_wheel_velocity(LEFT, cmd->linear_x, cmd->angular_z);
}
void on_right_cmd(const protocan_motor_command_t *cmd, void *ctx) {
    set_wheel_velocity(RIGHT, cmd->linear_x, cmd->angular_z);
}

int main(void) {
    canfd_hal_init();

    // デバイス初期化
    protocan_device_init(&device, "DualMotorBoard");
    protocan_device_set_id(&device, read_dip_switch());

    // ノード初期化 — instance_index と instance_name を C で指定
    // (device_manifest.toml の役割をここで担う)
    protocan_bldc_motor_init(&motor_left,  0, "left");
    protocan_bldc_motor_init(&motor_right, 1, "right");
    protocan_imu_init(&imu_node, 0, NULL);  // instance_name NULL → デフォルト

    // デバイスにノードを登録
    // → 内部で local_node_id が自動割当 (登録順: 0, 1, 2)
    // → ディスクリプタがランタイムで連結される
    protocan_device_add_node(&device, &motor_left.base);
    protocan_device_add_node(&device, &motor_right.base);
    protocan_device_add_node(&device, &imu_node.base);

    // コールバック登録
    protocan_bldc_motor_on_motor_command(&motor_left,  on_left_cmd,  NULL);
    protocan_bldc_motor_on_motor_command(&motor_right, on_right_cmd, NULL);

    // 開始 — ハートビートにはデバイス名・全ノードの schema_hash が含まれる
    protocan_device_start(&device);

    while (1) {
        protocan_device_poll(&device);

        if (protocan_device_state(&device) == PROTOCAN_STATE_OPERATIONAL) {
            motor_left.encoder_fb.position_cnt = read_encoder(LEFT);
            motor_left.encoder_fb.velocity_cps = read_encoder_vel(LEFT);
            motor_right.encoder_fb.position_cnt = read_encoder(RIGHT);
            motor_right.encoder_fb.velocity_cps = read_encoder_vel(RIGHT);

            imu_node.imu_data.accel_x = read_accel_x();
            imu_node.imu_data.accel_y = read_accel_y();
            imu_node.imu_data.accel_z = read_accel_z();
            imu_node.imu_data.gyro_x  = read_gyro_x();
            imu_node.imu_data.gyro_y  = read_gyro_y();
            imu_node.imu_data.gyro_z  = read_gyro_z();

            protocan_bldc_motor_publish_encoder_feedback(&motor_left);
            protocan_bldc_motor_publish_encoder_feedback(&motor_right);
            protocan_bldc_motor_publish_motor_status(&motor_left);
            protocan_bldc_motor_publish_motor_status(&motor_right);
            protocan_imu_publish_imu_data(&imu_node);
        }
    }
}
```

### 4.6 デバイスディスクリプタの構築フロー

```
protoc-gen-protocan が生成:
  ┌────────────────────────┐  ┌──────────────────┐
  │ BLDC Motor Node Desc   │  │ IMU Node Desc    │
  │ (const ROM per .proto) │  │ (const ROM)      │
  └────────┬───────────────┘  └────────┬─────────┘
           │                           │
           └─────────┬─────────────────┘
                     │ protocan_device_add_node() が呼ばれるたび
                     ▼
  ┌──────────────────────────────────────────────────────────┐
  │ Device Descriptor (ランタイムで連結)                       │
  │ ┌─ Header: "DualMotorBoard", num_nodes=3               │
  │ ├─ Node[0]: BLDCMotor, local=0, inst=0, name="left"   │
  │ │   └─ topics, params, ros2 mappings...                │
  │ ├─ Node[1]: BLDCMotor, local=1, inst=1, name="right"  │
  │ │   └─ (同上)                                          │
  │ └─ Node[2]: IMU, local=2, inst=0, name=NULL            │
  │     └─ topics, params, ros2 mappings...                │
  └──────────────────────────────────────────────────────────┘
```

protocan ランタイムは `protocan_device_start()` 時に各ノードの
ディスクリプタ片をシリアライズし、バルク転送用バッファを構築する。

---

## 5. デバイスディスクリプタ (バイナリ形式)

### 5.1 全体構造

```
┌──────────────────────────────────────────────────────────┐
│ Header (8 bytes)                                         │
│   [0:1]  magic           = 0x50CA                        │
│   [2]    descriptor_ver  = 4                             │
│   [3]    num_nodes       : uint8                         │
│   [4:5]  total_length    : uint16 LE                     │
│   [6:7]  crc16           : uint16 LE                     │
├──────────────────────────────────────────────────────────┤
│ Device Name: len-prefixed UTF-8                          │
├──────────────────────────────────────────────────────────┤
│ Node Descriptors × num_nodes                             │
└──────────────────────────────────────────────────────────┘
```

### 5.2 ノードディスクリプタ

```
Per Node:
┌──────────────────────────────────────────────────────────┐
│ [0:3]  schema_hash      : uint32 LE                      │
│ [4]    local_node_id    : uint8                          │
│ [5]    instance_index   : uint8                          │
│ [6]    num_topics       : uint8                          │
│ [7]    num_params       : uint8                          │
│ [..]   node_type_name   : len-prefixed UTF-8             │
│ [..]   instance_name    : len-prefixed UTF-8 (空可)      │
│ [..]   ros2_namespace   : len-prefixed UTF-8             │
├──────────────────────────────────────────────────────────┤
│ Topic Descriptors × num_topics                           │
├──────────────────────────────────────────────────────────┤
│ Param Descriptors × num_params                           │
└──────────────────────────────────────────────────────────┘
```

### 5.3 トピックディスクリプタ

```
Per Topic:
┌──────────────────────────────────────────────────────────┐
│ [0]    topic_index    : uint8                            │
│ [1]    direction      : uint8 (0=D2B, 1=B2D)            │
│ [2]    periodic       : uint8 (bool)                     │
│ [3]    priority       : uint8                            │
│ [4]    total_size     : uint8 (packed bytes)             │
│ [5]    num_fields     : uint8                            │
│ [..]   topic_name     : len-prefixed UTF-8               │
│ [..]   ros2_msg_type  : len-prefixed UTF-8  ★ (空 = カスタム) │
│ [..]   ros2_topic     : len-prefixed UTF-8  ★ (空 = デフォルト) │
├──────────────────────────────────────────────────────────┤
│ Field Descriptors × num_fields                           │
│   [0]    field_type   : uint8                            │
│   [1]    offset       : uint8 (packed 内位置)            │
│   [2]    size         : uint8                            │
│   [..]   field_name   : len-prefixed UTF-8               │
│   [..]   ros2_field   : len-prefixed UTF-8  ★ (空 = field_name) │
└──────────────────────────────────────────────────────────┘
```

> ★ = v0.4 で追加。ブリッジはこの情報だけで ROS 2 トピックを構築できる。

### 5.4 サイズ見積もり

```
DualMotorBoard (BLDCMotor×2 + IMU) の場合:

Header:             8 bytes
Device name:        1 + 15 = 16 bytes ("DualMotorBoard")

BLDCMotor Node (×2):
  Node header:      4+1+1+1+1 + (1+9) + (1+5) + (1+5) = 30 bytes
  Topic "motor_status":
    Header:         6 + (1+12) + (1+0) + (1+0) = 21 bytes
    4 fields:       4 × (3 + (1+~12) + (1+0)) ≈ 68 bytes
  Topic "encoder_feedback": ≈ 50 bytes
  Topic "motor_command":
    Header:         ≈ 40 bytes (ros2_msg_type="geometry_msgs/msg/Twist")
    6 fields:       6 × (3 + (1+~9) + (1+~9)) ≈ 138 bytes
  Params: ≈ 60 bytes
  Per BLDCMotor node: ≈ 400 bytes
  ×2 = 800 bytes

IMU Node:
  Node + 1 topic (6 fields): ≈ 300 bytes

Total: ≈ 8 + 16 + 800 + 300 = ~1124 bytes
CAN FD 64B フレーム: 約 18 フレーム
```

> v0.3 までの ~300 bytes から増加するが、初回接続時に 1 回だけの転送なので許容範囲。
> schema_hash キャッシュにより 2 回目以降は転送不要。

---

## 6. CAN フレーム設計

（v0.3 から変更なし）

### 6.1 二層構成

| 層 | CAN ID 形式 | 用途 |
|----|------------|------|
| **データ層** | Standard ID (11-bit) | PDO — マスター割当、高速 |
| **管理層** | Extended ID (29-bit) | NMT, DISC, PARAM, PDO_CFG, BULK, SYNC |

### 6.2 管理層: Extended ID (29-bit)

```
Bit [28:25] Function Code (4b)
Bit [24:18] Source Device ID (7b)
Bit [17:14] Source Local Node (4b)
Bit [13:0]  Sub-field (14b)
```

### 6.3 データ層: Standard ID (11-bit)

```
PDO ID = 0x001–0x7FF (マスターが割当)
  0x001–0x0FF : 高優先度
  0x100–0x3FF : 中優先度
  0x400–0x7FF : 低優先度
```

---

## 7. PDO マッピング / デバイス間ルーティング

（v0.3 から変更なし — 詳細は v0.3 セクション 5, 6 参照）

- マスターが PDO_CFG で全 PDO を設定
- ノード・トピック横断のフレームパッキング
- TX/RX PDO ペアリングによるデバイス間直接通信

---

## 8. プロトコル詳細

### 8.1 ハートビート (NMT)

```
CAN ID: EXT_ID(NMT, device_id, 0, 0x0001)

Payload (最大 64 bytes):
  [0]     state         : uint8  (BOOT=0, PREOP=1, OP=2, STOPPED=3)
  [1]     num_nodes     : uint8
  [2:5]   uptime_ms     : uint32 LE
  Per node (num_nodes 回, 各 8 bytes):
    [+0:3] schema_hash  : uint32 LE
    [+4]   local_node_id: uint8
    [+5]   instance_index: uint8
    [+6:7] (reserved)   : uint16
```

### 8.2 ディスカバリフロー

```
┌────────┐                              ┌──────────┐
│ Master │                              │  Device  │
└───┬────┘                              └─────┬────┘
    │                                         │
    │  ◄── HEARTBEAT ────────────────────────│  schema_hash ×N
    │                                         │
    │  全 hash がキャッシュにある？               │
    │  ┌─ YES: キャッシュからセットアップ          │
    │  │                                      │
    │  └─ NO:                                 │
    │── DISC_GET_DESCRIPTOR ────────────────► │
    │  ◄── BULK_TRANSFER ───────────────────│  ディスクリプタ (ROS 2 情報含む)
    │                                         │
    │  ※ ディスクリプタをキャッシュに保存          │
    │  ※ ros2_msg_type / ros2_field を解析      │
    │  ※ PDO マッピング計算                      │
    │                                         │
    │── PDO_CFG ×N ─────────────────────────►│
    │  ◄── PDO_CFG_ACK ────────────────────│
    │── NMT_START ──────────────────────────►│
    │                                         │
    │  ※ ROS 2 トピック/パラメータを動的生成      │
    │                                         │
    │  ◄══ PDO DATA (Standard ID) ══════════│
```

---

## 9. protocan_bridge (汎用 ROS 2 ノード)

### 9.1 設計原則

- **コード生成なし**: デバイスのスキーマに依存するコードは一切含まない
- **ディスクリプタ駆動**: 全ての ROS 2 インターフェースをディスクリプタから動的構築
- **rosidl_typesupport_introspection**: フィールド名ベースで ROS 2 メッセージを操作

### 9.2 動的メッセージ操作

ROS 2 の `rosidl_typesupport_introspection_cpp` を使用し、
メッセージ型の構造体をランタイムで走査・フィールドアクセスする。

```cpp
// 概念的な疑似コード

class DynamicMsgWriter {
public:
    DynamicMsgWriter(const std::string& msg_type)
    {
        // "sensor_msgs/msg/Imu" → ランタイムで型情報を取得
        lib_ = load_typesupport_library(msg_type);
        members_ = get_introspection_members(lib_, msg_type);
        msg_buf_.resize(members_->size_of_);
        members_->init_function_(msg_buf_.data(),
                                 rosidl_runtime_cpp::MessageInitialization::ZERO);
    }

    // ドット区切りパスでフィールドに書き込み
    // "linear_acceleration.x" → msg.linear_acceleration.x
    void set_field(const std::string& path, double value)
    {
        auto [member, ptr] = resolve_path(members_, msg_buf_.data(), path);
        write_numeric(ptr, member->type_id_, value);
    }

    void* data() { return msg_buf_.data(); }

private:
    // "linear_acceleration.x" を再帰的に辿る
    std::pair<const MemberInfo*, void*>
    resolve_path(const Members* members, void* base,
                 const std::string& path)
    {
        auto dot = path.find('.');
        std::string head = path.substr(0, dot);

        for (uint32_t i = 0; i < members->member_count_; i++) {
            if (head == members->members_[i].name_) {
                void* field_ptr = static_cast<uint8_t*>(base)
                                + members->members_[i].offset_;
                if (dot == std::string::npos) {
                    return {&members->members_[i], field_ptr};
                }
                // ネスト構造体 → 再帰
                auto sub = get_sub_members(&members->members_[i]);
                return resolve_path(sub, field_ptr, path.substr(dot+1));
            }
        }
        throw std::runtime_error("field not found: " + path);
    }
};
```

### 9.3 ros2_msg_type 指定時の Publisher 構築

```cpp
void ProtoCANBridge::create_topic_from_descriptor(
    const NodeDescriptor& node,
    const TopicDescriptor& topic,
    const std::string& namespace_prefix)
{
    std::string full_topic = namespace_prefix + "/" + topic.name;

    if (!topic.ros2_msg_type.empty()) {
        // ──── 既存 ROS 2 型を使用 ────
        auto writer = std::make_shared<DynamicMsgWriter>(topic.ros2_msg_type);
        auto pub = create_generic_publisher(full_topic, topic.ros2_msg_type);

        register_pdo_callback(topic, [=](const uint8_t* pdo_data) {
            // ディスクリプタのフィールド情報に従い CAN → ROS 2 変換
            for (auto& field : topic.fields) {
                double value = decode_can_field(pdo_data, field);
                std::string target = field.ros2_field.empty()
                                   ? field.name
                                   : field.ros2_field;
                writer->set_field(target, value);
            }
            pub->publish(writer->serialized_message());
        });
    } else {
        // ──── カスタム動的型 ────
        // ディスクリプタのフィールド名・型から動的にメッセージを構築
        auto dynamic_pub = create_dynamic_publisher(full_topic, topic);
        register_pdo_callback(topic, [=](const uint8_t* pdo_data) {
            auto msg = build_dynamic_message(topic, pdo_data);
            dynamic_pub->publish(msg);
        });
    }
}
```

### 9.4 ros2_msg_type 省略時の動的メッセージ型

`ros2_msg_type` が空の場合、ブリッジはディスクリプタのフィールド情報から
動的にメッセージ型を構築する。方法は複数考えられる:

| 方式 | 説明 | メリット | デメリット |
|------|------|---------|----------|
| **A: key-value 配列** | `protocan_msgs/msg/DynamicFields` のような汎用型で、フィールド名と値のペア配列 | 実装が単純 | 型安全性が低い |
| **B: ランタイム .msg 生成** | ディスクリプタから `.msg` ファイルを生成し、ランタイムでコンパイル | 型安全 | 起動が遅い |
| **C: GenericPublisher** | `rclcpp::GenericPublisher` でシリアライズ済みバイト列を直接送信 | 柔軟 | Subscriber 側に型定義が必要 |

**推奨: 方式 A を基本とし、ユーザーが .msg を手動作成すれば方式 C に切替**

```
# protocan_msgs/msg/DynamicFields.msg
# 汎用動的メッセージ型
std_msgs/Header header
string[] field_names
float64[] field_values
uint8[] raw_data        # packed binary (オプション)
```

### 9.5 自動生成されるトピック階層

```
/protocan/
  ├── dual_motor_board_10/              # デバイス名_ID
  │   ├── left/                         # instance_name
  │   │   ├── motor_status              # (protocan_msgs/DynamicFields — カスタム型)
  │   │   ├── encoder_feedback          # (protocan_msgs/DynamicFields)
  │   │   ├── motor_command             # (geometry_msgs/msg/Twist — 既存型)
  │   │   └── Parameters:
  │   │       ├── pid_gains/*
  │   │       ├── device_info/*
  │   │       └── _period/motor_status_ms, encoder_feedback_ms
  │   ├── right/
  │   │   └── (同上)
  │   └── imu/
  │       └── imu_data                  # (sensor_msgs/msg/Imu — 既存型)
  │
  └── protocan_bridge/
      ├── Parameters:
      │   ├── can_interface
      │   ├── heartbeat_timeout_ms
      │   └── pdo_mapping_strategy
      └── Services:
          ├── list_devices
          ├── remap_pdo
          └── add_route
```

### 9.6 ブリッジ全体の動作フロー

```cpp
class ProtoCANBridge : public rclcpp::Node {
    SocketCAN can_;
    std::unordered_map<uint32_t, CachedDescriptor> desc_cache_;  // hash → desc
    std::unordered_map<uint16_t, PdoHandler> pdo_handlers_;      // std CAN ID → handler

public:
    ProtoCANBridge() : Node("protocan_bridge") {
        can_.open(get_parameter("can_interface").as_string());
        can_.set_ext_filter(FUNC_NMT, [this](auto& f){ on_heartbeat(f); });
    }

    void on_heartbeat(const CANFrame& frame) {
        auto device_id = extract_device_id(frame);
        auto nodes = parse_heartbeat_nodes(frame);

        bool all_cached = std::all_of(nodes.begin(), nodes.end(),
            [&](auto& n){ return desc_cache_.count(n.schema_hash); });

        if (all_cached) {
            setup_device(device_id, nodes);
        } else {
            request_descriptor(device_id);
            // → on_descriptor_received() に続く
        }
    }

    void on_descriptor_received(uint8_t device_id,
                                const DeviceDescriptor& desc) {
        // ディスクリプタをキャッシュ
        for (auto& node : desc.nodes) {
            desc_cache_[node.schema_hash] = node;
        }
        setup_device(device_id, desc.nodes);
    }

    void setup_device(uint8_t device_id, const std::vector<NodeInfo>& nodes) {
        // 1. PDO マッピング計算 & デバイスに設定
        auto pdos = compute_pdo_mapping(device_id, nodes);
        for (auto& pdo : pdos) send_pdo_config(device_id, pdo);

        // 2. ルーティング適用
        apply_routing(device_id);

        // 3. NMT START
        send_nmt_start(device_id);

        // 4. ROS 2 インターフェース構築 (ディスクリプタ情報のみで)
        for (auto& node : nodes) {
            auto& desc = desc_cache_[node.schema_hash];
            std::string ns = build_namespace(device_id, desc, node);
            for (auto& topic : desc.topics)
                create_topic_from_descriptor(desc, topic, ns);
            for (auto& param : desc.params)
                create_param_from_descriptor(desc, param, ns);
        }

        // 5. PDO 受信フィルタ登録
        for (auto& pdo : pdos)
            can_.set_std_filter(pdo.id, [this, pdo](auto& f){
                on_pdo_received(pdo, f);
            });
    }
};
```

---

## 10. アドレス管理

```
Device ID (7 bits):
  0x00       : ブロードキャスト
  0x01       : マスター (PC)
  0x02–0x0F  : 予約
  0x10–0x7E  : 一般デバイス (111 台)
  0x7F       : 未割当

Local Node ID (4 bits): 0x0–0xF
  protocan_device_add_node() の呼び出し順で自動割当
```

---

## 11. エラーハンドリング

| 状況 | マスター | デバイス |
|------|---------|---------|
| HB タイムアウト | stale 診断、PDO 無効化 | 自律動作 or 安全停止 |
| PDO_CFG 不整合 | ディスクリプタ再取得 → リトライ | NACK |
| hash 衝突 | ディスクリプタの full 比較 | — |
| 未知 ros2_msg_type | 警告ログ、DynamicFields にフォールバック | — |
| Bus-Off | 指数バックオフ | 同左 |

---

## 12. 設計判断のまとめ

| 項目 | v0.3 | v0.4 | 理由 |
|------|------|------|------|
| デバイス構成定義 | device_manifest.toml | **C/C++ ソースコード** | ビルドシステム簡素化、IDE 補完が効く |
| ブリッジ | コード生成あり | **汎用ランタイム** | デバイス追加時に再コンパイル不要 |
| ROS 2 メッセージ操作 | 生成コンバータ | **rosidl introspection 動的操作** | 任意の msg 型に実行時対応 |
| ディスクリプタ | CAN 情報のみ | **CAN + ROS 2 情報** | ブリッジがスキーマ事前知識不要 |
| スキーマ hash 入力 | CAN 構造のみ | **CAN + ROS 2 マッピング** | ディスクリプタの完全な同一性判定 |
| .msg 生成 | protoc が生成 | **不要** | 既存型再利用 or 動的型 |

v0.3 から継続:

- schema_hash (FNV-1a 32-bit) によるディスクリプタキャッシュ
- Standard ID データ層 / Extended ID 管理層
- PDO マッピング & TX/RX ペアリングルーティング
- periodic フラグのみ（周期はランタイムパラメータ）
- topic_index / local_node_id の自動割当

---

## 13. 今後の検討事項

- [ ] `protoc-gen-protocan` プラグイン実装
- [ ] protocan_bridge 汎用 C++ 実装
- [ ] `rosidl_typesupport_introspection` を使った DynamicMsgWriter の実装
- [ ] ros2_msg_type 省略時の動的型方式の最終決定 (A/B/C)
- [ ] ディスクリプタ圧縮（文字列の辞書化等）によるサイズ削減
- [ ] PDO マッピング最適化アルゴリズム
- [ ] SYNC タイムスタンプ同期
- [ ] ファームウェアアップデート over CAN FD
- [ ] Header.stamp の高精度タイムスタンプ設計
- [ ] 複数マスター冗長構成
- [ ] `oneof` / `repeated` の将来サポート
- [ ] ディスクリプタの Flash 永続化キャッシュ（マスター側）
