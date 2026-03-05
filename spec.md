# ProtoCAN — CAN FD プラグ・アンド・プレイ通信システム仕様 (最終版)

## 1. 概要

ProtoCAN は、CAN FD バス上で複数の組み込みデバイスと PC（ROS 2）が双方向通信を行うためのプロトコルである。

### 設計目標

- **プラグ・アンド・プレイ**: デバイス接続時に PC 側が自動で ROS 2 トピック・パラメータ・サービスを動的生成
- **低オーバーヘッド**: データフレームにタグやメタデータを含まない固定レイアウトエンコーディング (Packed Binary)
- **双方向・デバイス間通信**: CAN のブロードキャスト特性を活かし、PC 経由なしでデバイス同士も直接通信
- **スキーマ駆動**: `.proto` ファイルからファームウェア用 C ライブラリとコードを自動生成
- **自己記述デバイス**: 各デバイスがコンパクトなディスクリプタを内蔵し、接続時に自己宣言する。PC側は事前コンパイルなしでデバイスと通信可能。

### アーキテクチャ全体像

```
┌───────────────────────────────────────────────────────────────┐
│  .proto スキーマ群 (Node Type 定義)                             │
│  motor.proto, imu.proto, gpio.proto, ...                      │
└──────────┬────────────────────────────────────────────────────┘
           │  protoc-gen-protocan
           ▼
┌────────────────────────────────────────────────────────────┐
│  ファームウェア用 C ライブラリ (Node Type 単位で生成)              │
│  - packed 構造体 & エンコーダ/デコーダ                          │
│  - ディスクリプタ片 (const ROM) — ROS 2 マッピング情報を含む       │
│  - schema_hash 定数 (コンパイル時計算)                        │
└────────────┬───────────────────────────────────────────────┘
             │  ユーザーが C/C++ でデバイス構成を記述
             ▼
┌────────────────────────────────────────────────────────────┐
│  デバイスファームウェア (main.c)                                │
│  - protocan_device_add_node() でノード構成を宣言                │
│  - ディスクリプタのバイナリ blob は実行時に連結され転送可能になる   │
└────────────┬───────────────────────────────────────────────┘
             │
             ▼
┌───────────────────────────────────────────────────────────────┐
│                        CAN FD バス                             │
│  [Standard ID: PDO データ]    [Extended ID: 管理プロトコル]       │
└───────────────────────────────────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────────────────┐
│  protocan_bridge (汎用 ROS 2 ブリッジノード)                     │
│  - コンパイル済みスキーマや事前知識は一切不要                     │
│  - ディスクリプタを受信し、ROS 2 メッセージ/サービスを動的に構築    │
│  - schema_hash をキャッシュとして活用 (2回目以降は転送を省略)      │
└────────────────────────────────────────────────────────────┘
```

---

## 2. スキーマ定義 (.proto)

ProtoCAN は gRPC の `service` / `rpc` 構文を借用し、Topic (Pub/Sub)、Parameter (Read/Write)、Service (Req/Res) の CAN FD 上のデータフローを宣言的に表現する。

### 2.1 カスタムオプション定義

```protobuf
// protocan/options.proto
syntax = "proto3";
package protocan;

import "google/protobuf/descriptor.proto";

// ──── Service レベル = Node Type ────
message NodeOptions {
  string ros2_namespace = 1;  // 省略時は service 名から自動生成
}
extend google.protobuf.ServiceOptions {
  NodeOptions node = 50000;
}

// ──── Message レベル = ROS 2 型マッピング ────
message MsgOptions {
  // 既存 ROS 2 型の指定 (例: "sensor_msgs/msg/Imu")。
  // 省略時はブリッジが動的メッセージ型を構築する。
  string ros2_msg_type = 1;
}
extend google.protobuf.MessageOptions {
  MsgOptions msg = 50002;
}

// ──── Method レベル = Topic / Param / Service の振る舞い ────
message MethodOptions {
  bool   periodic       = 1;  // Topic: 周期送信か
  uint32 priority       = 2;  // Topic: CAN アービトレーション優先度 (0–7)
  bool   is_parameter   = 3;  // Parameter: この rpc を Parameter (R/W) として明示的に扱うフラグ
  bool   read_only      = 4;  // Parameter: 読み取り専用か
  string ros2_name      = 5;  // Topic/Param/Service の ROS 2 名オーバーライド
  string ros2_srv_type  = 6;  // Service: 既存 ROS 2 サービス型 (例: "std_srvs/srv/SetBool")
}
extend google.protobuf.MethodOptions {
  MethodOptions method = 50001;
}

// ──── Field レベル = ROS 2 フィールドパス ────
message FieldOptions {
  string ros2_field = 1;  // ROS 2 メッセージ内の対応フィールドパス (例: "linear.x")
}
extend google.protobuf.FieldOptions {
  FieldOptions field = 50003;
}
```

### 2.2 RPC パターンの解釈とセマンティクス

`protoc-gen-protocan` プラグインは `rpc` と `stream` の有無から以下を推論する:

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
│  (unary, is_parameter=false)          │ 要求/応答           │ Server        │
├───────────────────────────────────────────────────────────────────────────┤
│  rpc X(Req) returns (Res)             │ R/W Parameter      │ Parameter     │
│  (is_parameter=true)                  │ 読み書きパラメータ   │ (dynamic)     │
├───────────────────────────────────────────────────────────────────────────┤
│  rpc X(google.protobuf.Empty) returns (Msg) │ R/O Parameter      │ Parameter     │
│  (is_parameter=true, req=Empty)       │ 読み取り専用        │ (read-only)   │
└───────────────────────────────────────────────────────────────────────────┘
```

### 2.3 スキーマ定義例 (BLDCMotor と IMU)

```protobuf
// bldc_motor.proto
syntax = "proto3";
package bldc_motor;

import "google/protobuf/empty.proto";
import "protocan/options.proto";

message MotorStatus {
  float  current_a     = 1;
  float  velocity_rps  = 2;
  float  temperature_c = 3;
  uint32 error_flags   = 4;
}

// 既存の ROS 2 型にマッピングする例
message TwistCommand {
  option (protocan.msg) = { ros2_msg_type: "geometry_msgs/msg/Twist" };
  float linear_x  = 1 [(protocan.field) = { ros2_field: "linear.x" }];
  float angular_z = 6 [(protocan.field) = { ros2_field: "angular.z" }];
}

message SetEnableRequest {
  bool enable = 1 [(protocan.field) = { ros2_field: "data" }];
}
message SetEnableResponse {
  bool success = 1 [(protocan.field) = { ros2_field: "success" }];
}

service BLDCMotor {
  option (protocan.node) = { ros2_namespace: "motor" };

  // 1. Topic (TX)
  rpc Status(google.protobuf.Empty) returns (stream MotorStatus) {
    option (protocan.method) = { periodic: true, priority: 3 };
  }

  // 2. Topic (RX)
  rpc Command(stream TwistCommand) returns (google.protobuf.Empty) {
    option (protocan.method) = { priority: 2, ros2_name: "cmd_vel" };
  }

  // 3. Service (Req ≠ Res) - std_srvs/srv/SetBool に対応
  rpc SetEnable(SetEnableRequest) returns (SetEnableResponse) {
    option (protocan.method) = {
      ros2_srv_type: "std_srvs/srv/SetBool",
      ros2_name: "set_enable"
    };
  }
}
```

### 2.4 型マッピング（Proto型 → CAN バイナリ）

すべてのデータはシリアライズオーバヘッドをなくすため、フィールド番号順の **固定長 Packed Binary** 型で直接 CAN データフレーム (PDO) もしくはパラメータバルクに詰められる。

| Proto 型 | CAN バイナリ | サイズ | ROS 2 型 |
|----------|-------------|-------|---------|
| `bool`   | uint8 (0/1) | 1B | bool |
| `uint32` | LE u32 | 4B | uint32 |
| `int32`  | LE i32 | 4B | int32 |
| `float`  | IEEE 754 LE | 4B | float32 |
| `double` | IEEE 754 LE | 8B | float64 |
| `uint64` | LE u64 | 8B | uint64 |
| `int64`  | LE i64 | 8B | int64 |

*(※ v0.7 現在、データペイロードとしての `string`, `bytes`, `repeated`, `oneof` 等の可変長データは非サポート)*

---

## 3. スキーマハッシュ

### 3.1 構成と目的

- スキーマハッシュは、生成される **Node Type ごとの Descriptor Blob バイナリデータ列** を入力として **コンパイル時に計算** された 32-bit (FNV-1a) ハッシュ値である。入出力構造が単一のバイナリ（詳細は `descriptor_spec.md` を参照）に厳密に定まるため、ジェネレータの環境依存によるハッシュ揺らぎが発生しない。
- 複数の同じ Node Type のインスタンス（例: 2つのモーターノード）が存在する場合、それらは全く同じ `schema_hash` を持つ。
- これにより、`node_type_id` や `schema_version` といった ID を明示的に割り当てたり手動管理する必要がなくなる。
- ハートビート（NMT）にはデバイス上の全稼働ノードインスタンスの `schema_hash` が含まれ、PC側（ブリッジ）はそのハッシュを見て、未知のハッシュ種別であればディスクリプタを要求し、既知（キャッシュ済）であれば即座に通信を開始する。

---

## 4. CAN フレーム設計

ProtoCAN は帯域利用を最適化するため二層構成を採る。

| 層 | CAN ID 形式 | 用途 |
|----|------------|------|
| **データ層** | Standard ID (11-bit) | Topic データ (PDO) の送受信。マスターからの動的割当により優先度制御と高速転送を実現。 |
| **管理層** | Extended ID (29-bit) | プラグアンドプレイ機能 (NMT, DISC, PARAM, SERVICE, BULK 等)。 |

### 4.1 管理層: Extended ID (29-bit) レイアウト

管理層のメッセージルーティングを簡素化・一貫させるため、Extended ID は以下のビットフィールドで統一される。

```
Bit [28:25] Function Code (4 bit)
Bit [24:21] Source Device ID (4 bit) — 送信元 物理デバイス (0–15)
Bit [20:15] Source Local Node (6 bit) — 送信元 論理ノード (0–63)
Bit [14:11] Dest Device ID (4 bit) — 宛先 物理デバイス (0–15, 0xF はブロードキャスト)
Bit [10:5]  Dest Local Node (6 bit) — 宛先 論理ノード (0–63, 0x3F は全ノード/システム制御)
Bit [4:0]   Context / Seq (5 bit) — コマンドやシーケンス番号等 (コンテキスト依存)
```

**【Device ID の割り当てと衝突防止】**
デバイスIDは基板上のDIPスイッチ等で **物理的（静的）に設定** されることを前提とする。同一バス上に同じ Device ID を持つ物理ノードが存在するとデータの衝突やバスオフの原因となる。
これを防止するため、各デバイスは「起動直後に**ランダムな期間（例: 100〜500ms）**Listen-Only モードで待機し、自身と同じ Source Device ID を持つノードの NMT HEARTBEAT を観測した場合、直ちに ERROR 状態へ遷移して一切の送信を行わない（LED等で異常を通知する）」仕組みを実装することが要求される。

### 4.2 Function Code 一覧

| Code | 名称 | 概要 |
|------|------|------|
| 0x0 | NMT | ハートビート、状態管理 (BOOT / PREOP / OPERATIONAL) |
| 0x1 | EMCY | 緊急通知 |
| 0x2 | DISC | ディスカバリ（ディスクリプタ要求） |
| 0x3 | PARAM | R/W パラメータ読み書き要求 |
| 0x4 | PDO_CFG | PDO マッピング設定（マスター → デバイス） |
| 0x5 | BULK | ISO-TP 準拠の 64B 超データ分割転送 (ディスクリプタ等) |
| 0x6 | SYNC | マスターによる基準時刻配信 |
| 0x8 | SERVICE | ROS Service の Request/Response 呼び出し |

### 4.3 データ層: Standard ID (11-bit) と PDO

マスター（ブリッジ）はトピックの優先順位などに基づき、PDO ID（0x001 ~ 0x7FF）を割り当てる。

- **高優先度**: 0x001–0x0FF
- **中優先度**: 0x100–0x3FF
- **低優先度**: 0x400–0x7FF

異なるノードのトピック、あるいは同一ノードの複数トピックを 1 つの CAN フレーム片（最大64B）にまとめる **パッキング構成 (PDO マッピング)** により、バス帯域の消費を最小化する。

---

## 5. 管理層プロトコル詳細 (Extended ID)

本セクションでは、Function Code 別のパケット形式とプロトコルの詳細な動作シーケンスを規定する。

### 5.1 NMT (Network Management / 0x0)

デバイスの状態管理、およびマスターからの遷移司令プロトコルである。

```
■ HEARTBEAT (Device → Broadcast)
  CAN ID: EXT_ID(NMT, src_dev=device, src_node=0, dst_dev=0xF, dst_node=0x3F, ctx=0)
  Payload (最大 64 bytes):
    [0]     state         : uint8 (BOOT=0, PREOP=1, OP=2, ERROR=3, STOPPED=4)
    [1]     num_nodes     : uint8
    [2:5]   uptime_ms     : uint32 LE
    Per node (num_nodes 回, 各 8 bytes):
      [+0:3] schema_hash  : uint32 LE (Node Type ごとのハッシュ)
      [+4]   local_node_id: uint8 (0-63)
      [+5:7] (reserved)   : ゼロ埋め

※ OP (OPERATIONAL) 状態以降は帯域節約のため、`num_nodes = 0` としてベース部分（6 bytes）のみを送信する。PREOP 状態までは全ノードの情報を送信しきるまで複数のフレームを周期送信する。

■ NMT_CTRL (Master → Device)
  CAN ID: EXT_ID(NMT, src_dev=master, src_node=0, dst_dev=target, dst_node=0x3F, ctx=command)
  Context:
    0x1 = START (PREOP -> OP)
    0x2 = STOP  (OP -> STOPPED)
    0x3 = ENTER_PREOP (OP/STOPPED -> PREOP)
    0x4 = RESET_NODE (再起動)
  Payload: なし
```

### 5.2 EMCY (Emergency / 0x1)

デバイスのハードウェア障害や重大なエラーを非同期に通知する。

```
■ EMCY_MESSAGE (Device → Broadcast)
  CAN ID: EXT_ID(EMCY, src_dev=device, src_node=error_node, dst_dev=0xF, dst_node=0x3F, ctx=0)
  Payload:
    [0:1]  error_register : uint16 LE (内部エラーフラグ)
    [2..]  error_data     : packed binary (詳細エラー情報)
```

### 5.3 DISC (Discovery / 0x2) とブリッジ初期化

マスターがデバイスの持つスキーマを取得するためのプロトコル。
マスターは NMT `HEARTBEAT` で未知の `schema_hash` を検出すると、ディスカバリを実行する。

```
■ DISC_GET_DESCRIPTOR (Master → Device)
  CAN ID: EXT_ID(DISC, src_dev=master, src_node=0, dst_dev=target, dst_node=target_node, ctx=0)
  Payload: なし
```

※ デバイスからの応答となるディスクリプタバイナリは 64B を超えるため、**BULK 転送 (0x5)** を通じて当該 Node Type の Descriptor Blob がそのまま返答される。
※ マスターは `schema_hash` をキャッシュで一元管理するため、同じハッシュを持つ別の `local_node_id` に対しては要求をスキップする。

**初期化シーケンス:**
1. Master: `HEARTBEAT` 受信 (各ノードの Schema Hash と Local Node ID 群を確認)
2. Master: 未知のハッシュを持つ `local_node_id` に対して `DISC_GET_DESCRIPTOR` を送信
3. Device: `BULK TRANSFER` で要求されたノードの Descriptor Blob を返答
4. Master: 取得した Blob からトピック・サービス・パラメータ情報を復元
5. Master: `PDO_CFG` で PDO マッピングを送信し、全設定完了後に `NMT_CTRL (START)` を送信
6. Device: `OP` 状態へ遷移し、PDO 通信を開始

### 5.4 PARAM (Parameter Access / 0x3)

SDO のように、各ノードのパラメータの読み書きを1対1で行う。

```
■ PARAM_READ_REQ / PARAM_WRITE_REQ (Requester → Responder)
  CAN ID: EXT_ID(PARAM, src_dev=req, src_node=0, dst_dev=target, dst_node=target_node, ctx=command)
  Context: 0x01 (READ) または 0x02 (WRITE)
  Payload:
    [0]    param_index : uint8
    [1..]  value       : packed binary (WRITE の場合のみ)

■ PARAM_READ_RES / PARAM_WRITE_RES (Responder → Requester)
  CAN ID: EXT_ID(PARAM, src_dev=res, src_node=target_node, dst_dev=req, dst_node=0, ctx=command_res)
  Context: 0x81 (READ_RES) または 0x82 (WRITE_RES)
  Payload:
    [0]    param_index : uint8
    [1]    status      : uint8 (0=OK, 1=NOT_FOUND, 2=ERROR)
    [2..]  value       : packed binary (READ_RES で OK の場合のみ)
```

### 5.5 PDO_CFG (PDO Mapping Configuration / 0x4)

マスターがデバイスに対して、PDO (Standard CAN ID) の送受信マッピングを設定する。

```
■ PDO_CFG_TX (Master → Device)
  CAN ID: EXT_ID(PDO_CFG, src_dev=master, src_node=0, dst_dev=target, dst_node=0x3F, ctx=sequence)
  Context(sequence):
    0x00 = BEGIN
    0x01..0x1E = ENTRY
    0x1F = COMMIT

  Payload (BEGIN 時):
    [0:1]  pdo_id         : uint16 LE (Standard CAN ID, 0x001~0x7FF)
    [2]    direction      : uint8 (0=TX from device, 1=RX to device)
    [3]    num_entries    : uint8
    [4:5]  period_ms      : uint16 LE (TX周期, 0=イベント駆動)
    [6]    total_size     : uint8

  Payload (ENTRY 時):
    [0]    local_node_id  : uint8
    [1]    topic_index    : uint8
    [2]    field_index    : uint8  (0xFF = トピック全フィールド)
    [3]    offset         : uint8
    [4]    size           : uint8

  Payload (COMMIT 時):
    [0:1]  pdo_id         : uint16 LE
    [2]    action         : uint8 (0=Apply, 1=Delete)

■ PDO_CFG_ACK (Device → Master)
  CAN ID: EXT_ID(PDO_CFG, src_dev=device, src_node=0, dst_dev=master, dst_node=0x3F, ctx=0)
  Payload:
    [0:1]  pdo_id         : uint16 LE
    [2]    status         : uint8 (0=OK, 1=INVALID, 2=NO_RESOURCE)
```

### 5.6 BULK (Bulk Transfer / 0x5)

64B を超えるデータ（ディスクリプタ応答、FWアップデート、長大なサービス通信等）を ISO-TP (ISO 15765-2) 準拠で分割転送する。

```
■ BULK プロトコル
  CAN ID: EXT_ID(BULK, src_dev=device, src_node=node, dst_dev=target, dst_node=target_node, ctx=channel)
  ※ `channel` (5 bit) は同時進行する複数のバルク転送を区別するためのコンテキストID。

  First Frame:
    [0]   payload_type : uint8 (0=Descriptor, 1=Service Req, 2=Service Res, 0xFF=Other)
    [1:4] total_length : uint32 LE
    [5..] data[0..58]
  Consecutive Frame:
    [0]   sequence     : uint8 (1-255, 0 でラップ)
    [1..] data
  Flow Control (受信側 → 送信側):
    [0]   fc_flag      : uint8 (0=Continue, 1=Wait, 2=Abort)
    [1]   block_size   : uint8 (0=制限なし)
    [2:3] st_min_us    : uint16 LE (最小分離時間 μs)
```

### 5.7 SYNC (Synchronization / 0x6)

マスターからバス全体のタイムスタンプ同期や周期タスクの一斉開始を指示する信号。
※まだ実装しないこと

### 5.8 SERVICE (ROS Service Call / 0x8)

ROS 2 の Service 呼び出し（Request / Response）を実現する。

```
■ SERVICE_REQ (Master → Device)
  CAN ID: EXT_ID(SERVICE, src_dev=master, src_node=0, dst_dev=target, dst_node=target_node, ctx=sequence_id)
  Payload:
    [0]     service_index : uint8  (デバイス側 rpc 定義順)
    [1..]   request_data  : packed binary

■ SERVICE_RES (Device → Master)
  CAN ID: EXT_ID(SERVICE, src_dev=device, src_node=target_node, dst_dev=master, dst_node=0, ctx=sequence_id)
  Payload:
    [0]     service_index : uint8
    [1]     status        : uint8  (0=OK, 1=ERROR, 2=NOT_FOUND)
    [2..]   response_data : packed binary
```

※ リクエスト/レスポンスサイズが 64B を超え、単一パケットに収まらない場合は、自動的に **BULK 転送プロトコル** に切り替わる。
バルク転送時、First Frame の `data` 部は通常の SERVICE ペイロードから `[0] service_index` (RES時は `[0] service_index`, `[1] status`) を含めた形でシリアライズされる。`sequence_id` は BULK 転送においては `channel` (ctx) として扱われるか、バルク転送が提供するコンテキスト枠組みで担保される。

## 6. PDO マッピングとデバイス間直接ルーティング

### 6.1 デバイス間直接通信の概念

マスター（ROS 2 ブリッジ）はデータをPC経由で中継することなく、あるデバイスが出力したデータを別のデバイスの Rx 用トピックへ直接流し込むことができる（デバイス間直接ルーティング）。これにより、PCのCPU負荷やUSB/シリアルの通信遅延を完全に排除し、CANバス上の最小レイテンシ（数マイクロ秒〜数十マイクロ秒単位）で高速な制御ループを回すことが可能になる。

**動作の仕組み（例）:**
ジョイスティックノード (Device A) の出力をモーターノード (Device B) の目標速度トピックに直結させる場合:

- Device A には Topic `joy_status` を TX PDO `0x181` として設定
- Device B には Topic `cmd_vel` を RX PDO `0x181` として設定
- ペイロードフォーマットさえ一致（または互換性のある Packed Binary）していれば、Device B は自身の CAN FD ペリフェラルでハードウェアレベルで `0x181` を受け取り、即座に自身のコントローラへ適用する。

### 6.2 ROS 2 ブリッジの構成API（マスター側のインタフェース）

デバイス間ルーティングは ROS 2 サービス、または起動時の yaml パラメータ等でマスターに指示する。

**動的構成サービスの例 (`~/route_topic`):**
`protocan_bridge` ノードは `protocan_msgs/srv/RouteTopic` サービスを提供し、実行時にルーティング経路を切り替えられる。

```yaml
# Request
string source_topic  # 例: "/protocan/joy_board_01/buttons/status"
string dest_topic    # 例: "/protocan/dual_motor_05/left/cmd_vel"
---
# Response
bool success
string message
uint16 assigned_pdo_id
```

**マスター側の処理フロー:**

1. 双方の Topic に対応するスキーマ（データサイズと型）がバイナリレベルで互換性があるかを `schema_hash` や内部キャッシュから検証する。
2. 空いている CAN ID (例: `0x181`) を確保し、`PDO_CFG` (Function Code 0x4) を用いて送受信双方のデバイスに対して PDO マッピングを設定・適用（COMMIT）する。
3. デバイス間通信を設定した後も、マスター自身がその CAN フレーム (0x181) をスニッフィングし、ROS 2 上へ Publish し続ける（ロギング・監視用）といった挙動をオプションでオン/オフできる。

### 6.3 ファームウェア側の C API と挙動

デバイス間ルーティングが行われているか否かについて、デバイス側の開発者は**一切意識する必要がない（透過的である）**。
`protocan_device_poll()` の受信処理ループでマスターからの `PDO_CFG` コマンドがバックグラウンドで処理され、ペリフェラルの Rx マスクフィルタや受信テーブルが自動的に更新される。

```c
// Device B (Rx側) のファームウェア実装例：
// 通常のPCからの Topic サブスクライブと全く同じコードで済む。

int main(void) {
    // ... デバイスの初期化 ...
    
    // コールバックの登録
    protocan_bldc_motor_on_cmd_vel(&motor_left, on_cmd_vel_received, NULL);
    
    while (1) {
        // 受信・ハートビート送信・設定変更時のコマンド処理がここで行われる
        protocan_device_poll(&device);
        // ...
    }
}

// データの送信元がPCブリッジか、別デバイスか（直接ルーティングか）は意識しない
void on_cmd_vel_received(const protocan_twist_command_t *cmd, void *ctx) {
    // Device A (ジョイスティック等) から直接 1ms 周期などで飛んできたデータが処理される
    motor_set_velocity(LEFT, cmd->linear_x);
}
```

---

## 7. ディスクリプタ Blob とファームウェア実装

### 7.1 ディスクリプタ (Descriptor Blob)

ファームウェアに配置されるディスクリプタは、`protoc` プラグインが生成した **Node Type ごとの定数バイナリ配列** である (`const uint8_t[]`)。
ファームウェア自身はディスクリプタをパースせず、ブリッジからの DISC 要求に対して当該 `local_node_id` に対応する Descriptor Blob を応答として転送する。

ブリッジ側は受信した Blob をパースし、Node Type の内部構造（Topic, Service, Param）を動的に復元する。
> **Note**: Descriptor Blob のバイナリデータレイアウトの詳細仕様については、別紙 `descriptor_spec.md` を参照のこと。

### 7.2 C API コードフレームワーク

コード生成される `C API` により、開発者はスキーマの詳細を安全に使える。複数インスタンスの登録にも対応する。

```c
// ファームウェアイメージ例

#include "protocan_bldc_motor.h"
static protocan_device_t device;
static protocan_bldc_motor_node_t motor_left;   // インスタンス1
static protocan_bldc_motor_node_t motor_right;  // インスタンス2

// Service のコールバック実装
protocan_status_t on_set_enable(const protocan_set_enable_req_t *req,
                                protocan_set_enable_res_t *res, void *ctx) {
    // どのインスタンスに対するリクエストかは ctx または別途取得メソッドで判別可能
    motor_enable(LEFT, req->enable);
    res->success = true;
    return PROTOCAN_OK;
}

int main(void) {
    protocan_device_init(&device, "DualMotorBoard");

    // ノードの初期化と構成追加 (論理ノードID = 0, 1 として登録)
    protocan_bldc_motor_init(&motor_left, 0, "left");
    protocan_device_add_node(&device, &motor_left.base);
    
    protocan_bldc_motor_init(&motor_right, 1, "right");
    protocan_device_add_node(&device, &motor_right.base);

    protocan_bldc_motor_on_set_enable(&motor_left, on_set_enable, NULL);
    protocan_device_start(&device);

    while (1) {
        protocan_device_poll(&device); // 受信/ハートビート処理

        // Publish:
        motor_left.status.current_a = read_current(LEFT);
        protocan_status_t status = protocan_bldc_motor_publish_status(&motor_left);
        if (status != PROTOCAN_OK) {
            // エラーハンドリング
        }
    }
}
```
---

## 8. ROS 2 ブリッジノード概要

汎用設計の `protocan_bridge` ノードが一つ提供される。コード生成は不要。

- **Topics**: `ros2_msg_type` に標準/既存メッセージ指定があれば introspection にて自動解釈。省略時は `protocan_msgs/DynamicFields` 等による動的バインディングを実施。
- **Parameters**: `ros2 param list` 等で直接読み書き可能となる動的 ROS パラメータを展開。
- **Services**: `ros2_srv_type` がある場合、指定された型でコールバックサーバーを立ち上げる。受信時に CAN SERVICE_REQ にパッキング、タイムアウト等もブリッジが管理する。

```
/protocan/
  ├── dual_motor_board_10/
  │   ├── left/
  │   │   ├── cmd_vel        (Pub, geometry_msgs/msg/Twist)
  │   │   ├── status         (Sub, 動的生成)
  │   │   ├── Services:
  │   │   │   └── set_enable (std_srvs/srv/SetBool)
  │   │   └── Parameters:
  │   │       └── pid_gains/*
  └── protocan_bridge/
      └── Parameters/Services: can_interface, list_devices 等
```
