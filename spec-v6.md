# ProtoCAN — CAN FD プラグ・アンド・プレイ通信システム仕様 (Draft v0.6)

## 1. 概要

ProtoCAN は、CAN FD バス上で複数の組み込みデバイスと PC（ROS 2）が双方向通信を行うためのプロトコルである。

### v0.5 からの主な変更

- **ディスクリプタを不透明バイナリ blob 化**: ファームウェアはディスクリプタの内部構造を知らない。
  protoc プラグインが生成した `const uint8_t[]` をバルク転送で返すだけ
- **schema_hash をコンパイル時に計算**: ディスクリプタ blob の CRC/FNV をプラグインが算出し定数埋め込み
- **ROS 2 Service 対応**: `rpc X(Req) returns (Res)` で Req ≠ Res ≠ Empty → ROS Service にマッピング

---

## 2. RPC パターンの完全マッピング

### 2.1 一覧

```
┌───────────────────────────────────────────────────────────────────────────┐
│  rpc パターン                         │ ProtoCAN 意味      │ ROS 2 対応    │
├───────────────────────────────────────────────────────────────────────────┤
│  rpc X(..) returns (stream Msg)       │ TX Topic           │ Publisher     │
│  (server streaming)                   │ デバイスが publish   │               │
├───────────────────────────────────────────────────────────────────────────┤
│  rpc X(stream Msg) returns (..)       │ RX Topic           │ Subscription  │
│  (client streaming)                   │ デバイスが subscribe │               │
├───────────────────────────────────────────────────────────────────────────┤
│  rpc X(Req) returns (Res)             │ ROS Service        │ Service       │
│  (unary, Req ≠ Res, Req ≠ Empty)     │ 要求/応答           │ Server        │
├───────────────────────────────────────────────────────────────────────────┤
│  rpc X(Msg) returns (Msg)             │ R/W Parameter      │ Parameter     │
│  (unary, request型 == response型)     │ 読み書きパラメータ   │ (dynamic)     │
├───────────────────────────────────────────────────────────────────────────┤
│  rpc X(Empty) returns (Msg)           │ R/O Parameter      │ Parameter     │
│  (unary, request が Empty)            │ 読み取り専用        │ (read-only)   │
└───────────────────────────────────────────────────────────────────────────┘

判定ルール (protoc プラグイン, 優先順):
  1. stream あり → Topic
     - response が stream → TX (DEVICE_TO_BUS)
     - request が stream  → RX (BUS_TO_DEVICE)
  2. stream なし:
     a. request が Empty → R/O Parameter
     b. request型 == response型 → R/W Parameter
     c. request型 ≠ response型 → ROS Service
  ※ MethodOptions.read_only でオーバーライド可能
```

### 2.2 ROS Service の CAN 上の実現

ROS Service は Request/Response の 1 往復。CAN 管理層で実装:

```
■ SERVICE_REQ
  CAN ID: EXT_ID(SERVICE, requester_device, 0,
                 target_device << 4 | local_node)
  Payload:
    [0]     service_index : uint8  (rpc 定義順)
    [1]     sequence_id   : uint8  (req/res の対応付け)
    [2..]   request_data  : packed binary

■ SERVICE_RES
  CAN ID: EXT_ID(SERVICE, responder_device, local_node,
                 requester_device << 4)
  Payload:
    [0]     service_index : uint8
    [1]     sequence_id   : uint8
    [2]     status        : uint8  (0=OK, 1=ERROR, 2=NOT_FOUND)
    [3..]   response_data : packed binary

※ 64B を超える場合は BULK 転送にフォールバック
```

管理層 Function Code に SERVICE (0x8) を追加:

| Code | 名称 | 用途 |
|------|------|------|
| 0x0 | NMT | ハートビート、状態管理 |
| 0x1 | EMCY | 緊急通知 |
| 0x2 | DISC | ディスカバリ |
| 0x3 | PARAM | パラメータ読み書き |
| 0x4 | PDO_CFG | PDO マッピング設定 |
| 0x5 | BULK | バルク転送 |
| 0x6 | SYNC | 同期信号 |
| 0x7 | (予約) | |
| **0x8** | **SERVICE** | **ROS Service の req/res** |
| 0x9–0xF | (予約) | 将来拡張 |

---

## 3. スキーマ定義 (.proto)

### 3.1 カスタムオプション定義

```protobuf
// protocan/options.proto
syntax = "proto3";
package protocan;

import "google/protobuf/descriptor.proto";

// ──── Service レベル = Node Type ────

message NodeOptions {
  string ros2_namespace = 1;
}

extend google.protobuf.ServiceOptions {
  NodeOptions node = 50000;
}

// ──── Method レベル = Topic / Param / Service ────

message MethodOptions {
  bool   periodic       = 1;  // Topic: 周期送信
  uint32 priority       = 2;  // Topic: 優先度 (0–7)
  bool   read_only      = 3;  // Param: オーバーライド用

  string ros2_name      = 4;  // トピック/パラメータ/サービス名
  string ros2_msg_type  = 5;  // 既存 ROS 2 型
  string ros2_srv_type  = 6;  // 既存 ROS 2 サービス型
                               // (例: "std_srvs/srv/SetBool")
}

extend google.protobuf.MethodOptions {
  MethodOptions method = 50001;
}

// ──── Field レベル ────

message FieldOptions {
  string ros2_field = 1;  // "linear_acceleration.x" 等
}

extend google.protobuf.FieldOptions {
  FieldOptions field = 50003;
}
```

### 3.2 例: BLDC モーター (Topic + Param + Service)

```protobuf
// bldc_motor.proto
syntax = "proto3";
package bldc_motor;

import "google/protobuf/empty.proto";
import "protocan/options.proto";

// ──── データ型 ────

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

// ──── Service 用 Request/Response ────

message CalibrateRequest {
  uint32 mode = 1;  // 0=auto, 1=manual
  float  timeout_s = 2;
}

message CalibrateResponse {
  bool   success = 1;
  uint32 error_code = 2;
  float  offset_applied = 3;
}

message SetEnableRequest {
  bool enable = 1;
}

message SetEnableResponse {
  bool   success = 1;
  uint32 error_code = 2;
}

// ──── Node Type 定義 ────

service BLDCMotor {
  option (protocan.node) = { ros2_namespace: "motor" };

  // ──── Topics (stream あり) ────

  rpc Status(google.protobuf.Empty) returns (stream MotorStatus) {
    option (protocan.method) = { periodic: true, priority: 3 };
  }

  rpc Encoder(google.protobuf.Empty) returns (stream EncoderFeedback) {
    option (protocan.method) = { periodic: true, priority: 1 };
  }

  rpc Command(stream TwistCommand) returns (google.protobuf.Empty) {
    option (protocan.method) = {
      priority: 2,
      ros2_msg_type: "geometry_msgs/msg/Twist",
      ros2_name: "cmd_vel"
    };
  }

  // ──── Params (stream なし, 同型 or Empty→型) ────

  rpc Pid(PidGains) returns (PidGains) {}

  rpc Info(google.protobuf.Empty) returns (DeviceInfo) {}

  // ──── Services (stream なし, 異型) ────

  // キャリブレーション実行: Req ≠ Res → ROS Service
  rpc Calibrate(CalibrateRequest) returns (CalibrateResponse) {
    option (protocan.method) = { ros2_name: "calibrate" };
  }

  // モーター有効/無効: 既存 ROS 2 サービス型にマッピング
  rpc SetEnable(SetEnableRequest) returns (SetEnableResponse) {
    option (protocan.method) = {
      ros2_srv_type: "std_srvs/srv/SetBool",
      ros2_name: "set_enable"
    };
  }
}
```

### 3.3 例: IMU (Service 付き)

```protobuf
// imu.proto
syntax = "proto3";
package imu;

import "google/protobuf/empty.proto";
import "protocan/options.proto";

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

message ResetRequest {
  bool zero_gyro = 1;
  bool zero_accel = 2;
}

message ResetResponse {
  bool success = 1;
}

service IMU {
  option (protocan.node) = { ros2_namespace: "imu" };

  rpc Imu(google.protobuf.Empty) returns (stream ImuData) {
    option (protocan.method) = {
      periodic: true, priority: 1,
      ros2_msg_type: "sensor_msgs/msg/Imu"
    };
  }

  rpc Calibration(ImuCalibration) returns (ImuCalibration) {}

  rpc Info(google.protobuf.Empty) returns (ImuInfo) {}

  // Req ≠ Res → ROS Service
  rpc Reset(ResetRequest) returns (ResetResponse) {
    option (protocan.method) = { ros2_name: "reset" };
  }
}
```

### 3.4 RPC パターン判定の完全マッピング表

上記の `BLDCMotor` を例にとると:

| rpc 定義 | stream | Req型 | Res型 | 判定 | index |
|---------|--------|-------|-------|------|-------|
| `Status(Empty) returns (stream MotorStatus)` | res | Empty | MotorStatus | **TX Topic** | topic[0] |
| `Encoder(Empty) returns (stream EncoderFeedback)` | res | Empty | EncoderFeedback | **TX Topic** | topic[1] |
| `Command(stream TwistCommand) returns (Empty)` | req | TwistCommand | Empty | **RX Topic** | topic[2] |
| `Pid(PidGains) returns (PidGains)` | — | PidGains | PidGains | **R/W Param** | param[0] |
| `Info(Empty) returns (DeviceInfo)` | — | Empty | DeviceInfo | **R/O Param** | param[1] |
| `Calibrate(CalibrateReq) returns (CalibrateRes)` | — | CalibrateReq | CalibrateRes | **Service** | service[0] |
| `SetEnable(SetEnableReq) returns (SetEnableRes)` | — | SetEnableReq | SetEnableRes | **Service** | service[1] |

### 3.5 ros2_srv_type マッピング

`ros2_srv_type` 指定時、ブリッジは既存の ROS 2 サービス型で
サービスサーバーを公開する。フィールドマッピングは `ros2_field` で指定:

| `ros2_srv_type` | ブリッジの動作 |
|-----------------|-------------|
| **指定あり** | 既存 ROS 2 srv 型でサーバー作成。Request/Response を `ros2_field` でマッピング |
| **省略** | ディスクリプタのフィールド情報から動的にサービスを構築 |

```protobuf
// 例: std_srvs/srv/SetBool へのマッピング
message SetEnableRequest {
  bool enable = 1 [(protocan.field) = { ros2_field: "data" }];
  //                                     ↑ SetBool.Request.data
}
message SetEnableResponse {
  bool   success    = 1 [(protocan.field) = { ros2_field: "success" }];
  uint32 error_code = 2;
  //     ↑ error_code は ROS 側にマッピングなし → 無視
  // (SetBool.Response.message は string → CAN 側に対応なし → 空文字)
}
```

---

## 4. ディスクリプタ: 不透明バイナリ blob

### 4.1 設計原則

```
┌──────────────────────────────────────────────────────────────┐
│  protoc-gen-protocan (コンパイル時)                            │
│                                                              │
│  .proto → 解析 → ディスクリプタバイナリを生成                    │
│                  + schema_hash を計算                         │
│                  + ファームウェア用 C 構造体・API を生成          │
│                                                              │
│  出力:                                                        │
│    ┌───────────────────────────────────────────┐              │
│    │ protocan_bldc_motor.h / .c                │ ← FW が使う  │
│    │   packed 構造体、publish/subscribe API     │              │
│    │   #define SCHEMA_HASH 0x...               │              │
│    └───────────────────────────────────────────┘              │
│    ┌───────────────────────────────────────────┐              │
│    │ protocan_bldc_motor_desc.c                │ ← FW は     │
│    │   const uint8_t DESC_BLOB[] = { ... };    │   中身を     │
│    │   const size_t  DESC_BLOB_SIZE = ...;     │   知らない   │
│    └───────────────────────────────────────────┘              │
└──────────────────────────────────────────────────────────────┘

ファームウェア側:
  - DESC_BLOB を ROM に配置
  - SCHEMA_HASH をハートビートに含める
  - DISC 要求時に DESC_BLOB をバルク転送で返す
  - blob の内部構造を一切パースしない

ブリッジ側:
  - DESC_BLOB を受信 → パースして ROS 2 インターフェースを構築
  - schema_hash → DESC_BLOB のキャッシュ
```

### 4.2 ファームウェア側の生成コード

```c
// ============================================================
// protocan_bldc_motor.h (自動生成) — ファームウェアが使う部分
// ============================================================
#pragma once
#include "protocan/protocan.h"

// スキーマハッシュ (コンパイル時に計算済み)
#define BLDC_MOTOR_SCHEMA_HASH  0xA3F7B201u

// Topic / Param / Service Index
#define BLDC_MOTOR_TOPIC_STATUS    0
#define BLDC_MOTOR_TOPIC_ENCODER   1
#define BLDC_MOTOR_TOPIC_COMMAND   2
#define BLDC_MOTOR_PARAM_PID       0
#define BLDC_MOTOR_PARAM_INFO      1
#define BLDC_MOTOR_SERVICE_CALIBRATE  0
#define BLDC_MOTOR_SERVICE_SET_ENABLE 1

// ──── packed 構造体 (FW がデータアクセスに使う) ────

typedef struct __attribute__((packed)) {
    float    current_a;
    float    velocity_rps;
    float    temperature_c;
    uint32_t error_flags;
} protocan_motor_status_t;  // 16B

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

typedef struct __attribute__((packed)) {
    uint32_t mode;
    float    timeout_s;
} protocan_calibrate_req_t;  // 8B

typedef struct __attribute__((packed)) {
    uint8_t  success;
    uint32_t error_code;
    float    offset_applied;
} protocan_calibrate_res_t;  // 9B

typedef struct __attribute__((packed)) {
    uint8_t enable;
} protocan_set_enable_req_t;  // 1B

typedef struct __attribute__((packed)) {
    uint8_t  success;
    uint32_t error_code;
} protocan_set_enable_res_t;  // 5B

// ──── ノードハンドル ────

typedef struct {
    protocan_node_t base;

    // TX Topic バッファ
    protocan_motor_status_t      status;
    protocan_encoder_feedback_t  encoder;

    // RX Topic バッファ
    protocan_twist_command_t     command;

    // パラメータ
    protocan_pid_gains_t         pid;
    protocan_device_info_t       info;

    // Topic コールバック
    void (*on_command)(const protocan_twist_command_t*, void*);
    void *on_command_ctx;

    // Service コールバック
    protocan_status_t (*on_calibrate)(
        const protocan_calibrate_req_t *req,
        protocan_calibrate_res_t *res, void *ctx);
    void *on_calibrate_ctx;

    protocan_status_t (*on_set_enable)(
        const protocan_set_enable_req_t *req,
        protocan_set_enable_res_t *res, void *ctx);
    void *on_set_enable_ctx;
} protocan_bldc_motor_node_t;

// ──── API ────

void protocan_bldc_motor_init(
    protocan_bldc_motor_node_t *node,
    uint8_t instance_index,
    const char *instance_name);

// Publish
protocan_status_t protocan_bldc_motor_publish_status(
    protocan_bldc_motor_node_t *node);
protocan_status_t protocan_bldc_motor_publish_encoder(
    protocan_bldc_motor_node_t *node);

// Subscribe callback
void protocan_bldc_motor_on_command(
    protocan_bldc_motor_node_t *node,
    void (*cb)(const protocan_twist_command_t*, void*),
    void *ctx);

// Service callbacks
void protocan_bldc_motor_on_calibrate(
    protocan_bldc_motor_node_t *node,
    protocan_status_t (*cb)(const protocan_calibrate_req_t*,
                            protocan_calibrate_res_t*, void*),
    void *ctx);

void protocan_bldc_motor_on_set_enable(
    protocan_bldc_motor_node_t *node,
    protocan_status_t (*cb)(const protocan_set_enable_req_t*,
                            protocan_set_enable_res_t*, void*),
    void *ctx);
```

```c
// ============================================================
// protocan_bldc_motor_desc.c (自動生成) — 不透明バイナリ blob
// ============================================================
#include "protocan_bldc_motor.h"

// ブリッジだけがパースする。ファームウェアはこの中身を知らない。
// protoc プラグインがバイナリを直接出力。
const uint8_t PROTOCAN_BLDC_MOTOR_DESC_BLOB[] = {
    // --- Node Header ---
    // schema_hash: 0xA3F7B201 (LE)
    0x01, 0xB2, 0xF7, 0xA3,
    // node_type_name: "BLDCMotor" (len-prefixed)
    0x09, 'B','L','D','C','M','o','t','o','r',
    // ros2_namespace: "motor" (len-prefixed)
    0x05, 'm','o','t','o','r',
    // num_topics: 3, num_params: 2, num_services: 2
    0x03, 0x02, 0x02,

    // --- Topic[0]: Status ---
    // topic_index=0, direction=0(D2B), periodic=1, priority=3, total_size=16
    0x00, 0x00, 0x01, 0x03, 0x10,
    // name: "status" (len-prefixed)
    0x06, 's','t','a','t','u','s',
    // ros2_msg_type: "" (len=0, カスタム型)
    0x00,
    // ros2_name: "" (len=0, デフォルト)
    0x00,
    // num_fields: 4
    0x04,
    // field[0]: type=FLOAT32(0x0A), offset=0, size=4, name="current_a", ros2_field=""
    0x0A, 0x00, 0x04, 0x09, 'c','u','r','r','e','n','t','_','a', 0x00,
    // field[1]: ... (以降同様)
    // ...

    // --- Topic[2]: Command ---
    // ...
    // ros2_msg_type: "geometry_msgs/msg/Twist" (len-prefixed)
    0x17, 'g','e','o','m','e','t','r','y','_','m','s','g','s',
          '/','m','s','g','/','T','w','i','s','t',
    // ros2_name: "cmd_vel"
    0x07, 'c','m','d','_','v','e','l',
    // fields with ros2_field mappings:
    // field[0]: FLOAT32, 0, 4, "linear_x", "linear.x"
    // ...

    // --- Service[0]: Calibrate ---
    // service_index=0
    // name: "calibrate"
    // ros2_srv_type: "" (カスタム)
    // ros2_name: "calibrate"
    // request: num_fields=2, fields...
    // response: num_fields=3, fields...
    // ...

    // --- Service[1]: SetEnable ---
    // ros2_srv_type: "std_srvs/srv/SetBool"
    // request fields with ros2_field mapping
    // response fields with ros2_field mapping
    // ...
};

const size_t PROTOCAN_BLDC_MOTOR_DESC_BLOB_SIZE =
    sizeof(PROTOCAN_BLDC_MOTOR_DESC_BLOB);
```

### 4.3 ディスクリプタ blob のバイナリフォーマット

```
Node Type Descriptor Blob:
┌──────────────────────────────────────────────────────────┐
│ [0:3]   schema_hash      : uint32 LE                     │
│ [..]    node_type_name   : len-prefixed UTF-8            │
│ [..]    ros2_namespace   : len-prefixed UTF-8            │
│ [..]    num_topics       : uint8                         │
│ [..]    num_params       : uint8                         │
│ [..]    num_services     : uint8                         │
├──────────────────────────────────────────────────────────┤
│ Topic Descriptors × num_topics                           │
│   [0]    topic_index   : uint8                           │
│   [1]    direction     : uint8 (0=D2B, 1=B2D)           │
│   [2]    periodic      : uint8                           │
│   [3]    priority      : uint8                           │
│   [4]    total_size    : uint8 (packed bytes)            │
│   [..]   name          : len-prefixed UTF-8              │
│   [..]   ros2_msg_type : len-prefixed UTF-8 (空=カスタム) │
│   [..]   ros2_name     : len-prefixed UTF-8 (空=デフォルト)│
│   [..]   num_fields    : uint8                           │
│   Per Field:                                             │
│     [0]  field_type    : uint8                           │
│     [1]  offset        : uint8                           │
│     [2]  size          : uint8                           │
│     [..] field_name    : len-prefixed UTF-8              │
│     [..] ros2_field    : len-prefixed UTF-8 (空=同名)    │
├──────────────────────────────────────────────────────────┤
│ Param Descriptors × num_params                           │
│   (Topic と同構造 + read_only: uint8)                     │
├──────────────────────────────────────────────────────────┤
│ Service Descriptors × num_services                       │
│   [..]   service_index : uint8                           │
│   [..]   name          : len-prefixed UTF-8              │
│   [..]   ros2_srv_type : len-prefixed UTF-8 (空=カスタム) │
│   [..]   ros2_name     : len-prefixed UTF-8 (空=デフォルト)│
│   Request:                                               │
│     [..]   total_size  : uint8                           │
│     [..]   num_fields  : uint8                           │
│     Per Field: (Topic Field と同構造)                      │
│   Response:                                              │
│     [..]   total_size  : uint8                           │
│     [..]   num_fields  : uint8                           │
│     Per Field: (Topic Field と同構造)                      │
└──────────────────────────────────────────────────────────┘
```

### 4.4 デバイスディスクリプタ（デバイス全体）

デバイス全体のディスクリプタは、`protocan_device_add_node()` 時に
各ノードの blob を連結してランタイムで構築:

```
Device Descriptor (バルク転送される全体):
┌──────────────────────────────────────────────────────────┐
│ Header (8 bytes)                                         │
│   [0:1]  magic           = 0x50CA                        │
│   [2]    descriptor_ver  = 6                             │
│   [3]    num_nodes       : uint8                         │
│   [4:5]  total_length    : uint16 LE                     │
│   [6:7]  crc16           : uint16 LE                     │
├──────────────────────────────────────────────────────────┤
│ Device Name: len-prefixed UTF-8                          │
├──────────────────────────────────────────────────────────┤
│ Per Node Instance:                                       │
│   [0]    local_node_id   : uint8 (登録順で自動割当)       │
│   [1]    instance_index  : uint8                         │
│   [..]   instance_name   : len-prefixed UTF-8            │
│   [..]   Node Type Desc Blob (上記バイナリをそのまま連結)  │
└──────────────────────────────────────────────────────────┘
```

ファームウェアの `protocan_device_start()` は:

1. Header を構築（num_nodes, total_length, CRC を計算）
2. 各ノードの instance 情報 + blob を連結
3. 結果をバルク転送用バッファに格納

```c
// protocan_device.c (共通ランタイム) — 概念

void protocan_device_start(protocan_device_t *dev) {
    // ディスクリプタバッファを構築
    uint8_t *buf = dev->desc_buf;
    size_t pos = 0;

    // Header
    buf[pos++] = 0x50; buf[pos++] = 0xCA;  // magic
    buf[pos++] = 6;                          // ver
    buf[pos++] = dev->num_nodes;
    pos += 2;  // total_length (後で埋める)
    pos += 2;  // crc16 (後で埋める)

    // Device name
    pos += write_len_prefixed_str(buf + pos, dev->device_name);

    // Per node instance
    for (int i = 0; i < dev->num_nodes; i++) {
        protocan_node_t *node = dev->nodes[i];
        buf[pos++] = node->local_node_id;
        buf[pos++] = node->instance_index;
        pos += write_len_prefixed_str(buf + pos, node->instance_name);

        // Node Type の blob をそのままコピー (FW はこの中身を知らない)
        memcpy(buf + pos, node->desc_blob, node->desc_blob_size);
        pos += node->desc_blob_size;
    }

    // total_length と CRC を埋める
    write_u16_le(buf + 4, pos);
    write_u16_le(buf + 6, crc16(buf + 8, pos - 8));

    dev->desc_total_size = pos;

    // NMT 開始 (ハートビート送信)
    protocan_nmt_start(dev);
}
```

### 4.5 ハートビート

```
CAN ID: EXT_ID(NMT, device_id, 0, 0x0001)

Payload (最大 64 bytes):
  [0]     state         : uint8
  [1]     num_nodes     : uint8
  [2:5]   uptime_ms     : uint32 LE
  Per node (num_nodes 回, 各 8 bytes):
    [+0:3] schema_hash  : uint32 LE (コンパイル時計算済み)
    [+4]   local_node_id: uint8
    [+5]   instance_index: uint8
    [+6:7] (reserved)   : uint16
```

---

## 5. ファームウェア使用例

```c
// main.c — DualMotorBoard
#include "protocan/protocan.h"
#include "generated/protocan_bldc_motor.h"
#include "generated/protocan_imu.h"

static protocan_device_t device;
static protocan_bldc_motor_node_t motor_left, motor_right;
static protocan_imu_node_t imu_node;

// ── Topic コールバック ──
void on_left_cmd(const protocan_twist_command_t *cmd, void *ctx) {
    set_wheel_velocity(LEFT, cmd->linear_x, cmd->angular_z);
}

// ── Service コールバック ──
protocan_status_t on_left_calibrate(
    const protocan_calibrate_req_t *req,
    protocan_calibrate_res_t *res, void *ctx)
{
    float offset = run_calibration(LEFT, req->mode, req->timeout_s);
    res->success = (offset != 0.0f);
    res->error_code = res->success ? 0 : 1;
    res->offset_applied = offset;
    return PROTOCAN_OK;
}

protocan_status_t on_left_set_enable(
    const protocan_set_enable_req_t *req,
    protocan_set_enable_res_t *res, void *ctx)
{
    motor_enable(LEFT, req->enable);
    res->success = true;
    res->error_code = 0;
    return PROTOCAN_OK;
}

int main(void) {
    canfd_hal_init();

    protocan_device_init(&device, "DualMotorBoard");
    protocan_device_set_id(&device, read_dip_switch());

    protocan_bldc_motor_init(&motor_left,  0, "left");
    protocan_bldc_motor_init(&motor_right, 1, "right");
    protocan_imu_init(&imu_node, 0, NULL);

    protocan_device_add_node(&device, &motor_left.base);
    protocan_device_add_node(&device, &motor_right.base);
    protocan_device_add_node(&device, &imu_node.base);

    // Topic コールバック
    protocan_bldc_motor_on_command(&motor_left,  on_left_cmd,  NULL);
    protocan_bldc_motor_on_command(&motor_right, on_right_cmd, NULL);

    // Service コールバック
    protocan_bldc_motor_on_calibrate(&motor_left,  on_left_calibrate,  NULL);
    protocan_bldc_motor_on_calibrate(&motor_right, on_right_calibrate, NULL);
    protocan_bldc_motor_on_set_enable(&motor_left,  on_left_set_enable,  NULL);
    protocan_bldc_motor_on_set_enable(&motor_right, on_right_set_enable, NULL);

    protocan_device_start(&device);

    while (1) {
        protocan_device_poll(&device);
        // ... publish 処理 (v0.5 と同じ)
    }
}
```

---

## 6. protocan_bridge の Service 対応

### 6.1 自動生成されるインターフェース

```
/protocan/
  ├── dual_motor_board_10/
  │   ├── left/
  │   │   ├── status              (Sub, protocan_msgs/DynamicFields)
  │   │   ├── encoder             (Sub, protocan_msgs/DynamicFields)
  │   │   ├── cmd_vel             (Pub, geometry_msgs/msg/Twist)
  │   │   ├── Services:
  │   │   │   ├── calibrate       (カスタム動的型 or 生成型)
  │   │   │   └── set_enable      (std_srvs/srv/SetBool)
  │   │   └── Parameters:
  │   │       ├── pid/*
  │   │       ├── info/*
  │   │       └── _period/*
  │   └── right/
  │       └── (同上)
```

### 6.2 Service のブリッジ処理

```cpp
void ProtoCANBridge::create_service_from_descriptor(
    const NodeDescriptor& node,
    const ServiceDescriptor& svc,
    const std::string& ns)
{
    std::string full_name = ns + "/" + svc.ros2_name_or_default();

    if (!svc.ros2_srv_type.empty()) {
        // 既存 ROS 2 サービス型
        auto server = create_generic_service(
            full_name, svc.ros2_srv_type,
            [=](const void* req_ros, void* res_ros) {
                // ROS Request → CAN packed binary
                auto can_req = convert_ros_to_can(
                    req_ros, svc.request_fields, /*reverse mapping*/);

                // CAN SERVICE_REQ 送信 → SERVICE_RES 待ち
                auto can_res = send_service_request(
                    node.device_id, node.local_node_id,
                    svc.service_index, can_req);

                // CAN Response → ROS Response
                convert_can_to_ros(can_res, svc.response_fields, res_ros);
            });
    } else {
        // カスタム動的サービス型
        // (Topic の DynamicFields 方式と同様のアプローチ)
    }
}
```

### 6.3 Service の CAN シーケンス

```
┌──────────────┐        ┌───────────┐        ┌──────────┐
│  ROS Client  │        │  Bridge   │        │  Device  │
└──────┬───────┘        └─────┬─────┘        └─────┬────┘
       │                      │                     │
       │─ srv request ──────►│                     │
       │                      │                     │
       │                      │── SERVICE_REQ ────►│
       │                      │   (Extended ID)     │
       │                      │                     │
       │                      │   FW: on_calibrate()│
       │                      │   コールバック実行     │
       │                      │                     │
       │                      │◄── SERVICE_RES ────│
       │                      │                     │
       │◄─ srv response ─────│                     │
```

---

## 7. 設計判断のまとめ

### v0.6 の変更

| 項目 | v0.5 | v0.6 | 理由 |
|------|------|------|------|
| ディスクリプタ格納 | C 構造体配列 | **不透明 `uint8_t[]` blob** | FW の関心分離、将来の圧縮容易 |
| schema_hash 計算 | (未定) | **コンパイル時に確定** | FW のランタイムコスト排除 |
| ROS Service | 非対応 | **`Req≠Res≠Empty` パターン** | RPC 構文の自然な拡張 |
| Function Code | 7 種 | **8 種 (+SERVICE)** | Service の req/res 転送 |
| Service コールバック | — | **request→response 関数** | Topic コールバックと対称的な API |
| ros2_srv_type | — | **MethodOptions に追加** | 既存 ROS 2 srv 型の再利用 |

### 全バージョン継続事項

- Standard ID データ層 / Extended ID 管理層 (v0.2)
- PDO マッピング & TX/RX ルーティング (v0.2)
- schema_hash キャッシュ (v0.3)
- ros2_msg_type + ros2_field マッピング (v0.3)
- C API デバイス構成 (v0.4)
- 汎用ブリッジ、ディスクリプタ駆動 (v0.4)
- service/rpc 構文 (v0.5)

---

## 8. 今後の検討事項

- [ ] `protoc-gen-protocan` プラグイン実装
- [ ] protocan_bridge 汎用 C++ 実装
- [ ] Service のタイムアウト・リトライポリシー
- [ ] Service の 64B 超え時のバルク転送連携
- [ ] ディスクリプタ blob の圧縮 (LZ4/LZSS 等)
- [ ] ros2_srv_type 省略時の動的サービス型生成
- [ ] bidirectional stream の将来検討
- [ ] PDO マッピング最適化アルゴリズム
- [ ] SYNC タイムスタンプ同期
- [ ] ファームウェアアップデート over CAN FD
- [ ] 複数マスター冗長構成
