# protoc-gen-protocan — コードジェネレータ仕様

## 1. 概要

`protoc-gen-protocan` は `protoc` のプラグインとして動作するコードジェネレータである。
`.proto` ファイル内の `service` 定義を解釈し、ProtoCAN デバイス向け C++ ライブラリコードを自動生成する。

### 生成物の配置

```
入力: bldc_motor.proto
         ↓ protoc --protocan_out=... bldc_motor.proto
出力: bldc_motor.hpp   ← メッセージ struct、Node クラス宣言、スキーマ定数
      bldc_motor.cpp   ← encode/decode 実装、Descriptor Blob 配列、Node メソッド実装
```

1 つの `.proto` ファイルに複数の `service` が存在する場合、`service` ごとに 1 組の `.hpp` / `.cpp` を生成する。

### パイプライン全体における位置づけ

```
.proto スキーマ
  │
  ├─ protoc --cpp_out   → descriptor.pb.h / .cc  (マスター側ランタイムが使用)
  │
  └─ protoc --protocan_out
               │
               ├─ <snake_name>.hpp / .cpp  ← このドキュメントの対象 (デバイス側)
               └─ (将来: マスター側スタブ)
```

---

## 2. 入力仕様

### 2.1 前提とする Proto 構文

`syntax = "proto3";` のみをサポートする。

インポートが必要なファイル:

```protobuf
import "google/protobuf/empty.proto";   // Empty 型を使う場合
import "protocan/options.proto";         // カスタムオプション使用時
```

### 2.2 カスタムオプション一覧

| スコープ | オプション | フィールド | 型 | 説明 |
|---------|------------|----------|----|------|
| Service | `protocan.node` | `ros2_namespace` | string | ノードの ROS 2 名前空間。未指定時は service 名を snake_case に変換して使用 |
| Message | `protocan.msg` | `ros2_msg_type` | string | 対応する ROS 2 メッセージ型 (例: `"geometry_msgs/msg/Twist"`) |
| Method | `protocan.method` | `periodic` | bool | TX Topic: 周期送信フラグ |
| | | `priority` | uint32 | CAN アービトレーション優先度 (0–7) |
| | | `is_parameter` | bool | この rpc を Parameter として扱うフラグ |
| | | `read_only` | bool | Parameter: 読み取り専用 |
| | | `ros2_name` | string | ROS 2 上での名前オーバーライド |
| | | `ros2_srv_type` | string | Service: 対応する ROS 2 サービス型 |
| Field | `protocan.field` | `ros2_field` | string | ROS 2 メッセージ内のフィールドパス (例: `"linear.x"`) |
| | | `size` | uint32 | CAN 上でのバイトサイズ縮小 (1 または 2 のみ有効) |

### 2.3 RPC パターンの解釈

| rpc パターン | ProtoCAN 意味 | 生成物 |
|------------|--------------|-------|
| `rpc X(..) returns (stream Msg)` | TX Topic (デバイスが publish) | `<msg>_buffer()` + `publish_<x>()` |
| `rpc X(stream Msg) returns (..)` | RX Topic (デバイスが subscribe) | `on_<x>(Callback, void*)` |
| `rpc X(Req) returns (Res)` (is_parameter=false) | Service | `on_<x>(Callback, void*)` |
| `rpc X(Req) returns (Res)` (is_parameter=true) | R/W Parameter | `on_get_<x>` + `on_set_<x>` |
| `rpc X(google.protobuf.Empty) returns (Res)` (is_parameter=true) | R/O Parameter | `on_get_<x>` のみ |

---

## 3. 型マッピング

### 3.1 Proto フィールド型 → C++ 型 → Packed Binary

| Proto 型 | protocan.field.size | C++ 型 | バイト数 | エンコーディング |
|----------|---------------------|--------|---------|----------------|
| `bool` | — | `bool` | 1 | 0/1 の uint8 |
| `uint32` | 1 | `uint8_t` | 1 | LE u8 |
| `int32` | 1 | `int8_t` | 1 | LE i8 |
| `uint32` | 2 | `uint16_t` | 2 | LE u16 |
| `int32` | 2 | `int16_t` | 2 | LE i16 |
| `uint32` | 未指定 | `uint32_t` | 4 | LE u32 |
| `int32` | 未指定 | `int32_t` | 4 | LE i32 |
| `float` | — | `float` | 4 | IEEE 754 LE |
| `double` | — | `double` | 8 | IEEE 754 LE |
| `uint64` | — | `uint64_t` | 8 | LE u64 |
| `int64` | — | `int64_t` | 8 | LE i64 |

**制約**: `string`, `bytes`, `repeated`, `oneof`, `map` は非サポート。これらが含まれるメッセージに対してはコンパイルエラーを出力する。

### 3.2 フィールド順とオフセット計算

フィールドは **proto フィールド番号の昇順** に並べる。オフセットは前フィールドまでのサイズ累積。

```
MotorStatus { current_a(4B) + velocity_rps(4B) + temperature_c(4B) + error_flags(1B) }
→ offset: 0, 4, 8, 12
→ PACKED_SIZE = 13
```

---

## 4. 生成コード仕様

### R1 — メッセージ構造体

各メッセージ (`message` 定義) につき以下の構造体を生成する。

```cpp
// <snake_name>.hpp
#pragma once
#include <cstdint>
#include <cstring>

namespace protocan::<package> {

struct <MsgName> {
    // proto フィールド番号昇順。型は §3.1 マッピングに従う
    <C++型> <field_name>;
    ...

    static constexpr size_t PACKED_SIZE = <全フィールドバイト数合計>;

    /// Packed Binary (LE) にエンコード。UB 回避のため memcpy を使用する
    void encode(uint8_t* buf) const;

    /// Packed Binary (LE) からデコード
    static <MsgName> decode(const uint8_t* buf);
};

} // namespace protocan::<package>
```

**encode/decode の実装要件:**

- `memcpy` ベースのエンコード/デコードを使用し、未定義動作 (UB) を回避する
- エンディアン変換はフィールドごとにビットシフト演算で実装する
- `float` / `double` は `memcpy` で型パンニングする

```cpp
// 生成される encode の例 (float フィールド)
void MotorStatus::encode(uint8_t* buf) const {
    uint32_t tmp;
    std::memcpy(&tmp, &current_a, 4);
    buf[0] = tmp & 0xFF; buf[1] = (tmp >> 8) & 0xFF;
    buf[2] = (tmp >> 16) & 0xFF; buf[3] = (tmp >> 24) & 0xFF;
    // ... 以降のフィールド
}

static MotorStatus MotorStatus::decode(const uint8_t* buf) {
    MotorStatus msg{};
    uint32_t tmp = uint32_t(buf[0]) | (uint32_t(buf[1]) << 8)
                 | (uint32_t(buf[2]) << 16) | (uint32_t(buf[3]) << 24);
    std::memcpy(&msg.current_a, &tmp, 4);
    // ...
    return msg;
}
```

---

### R2 — Node クラス

`service` 定義につき 1 つの `Node` クラスを生成する。`protocan::device::NodeBase` を継承する。

```cpp
// <snake_name>.hpp

class Node : public protocan::device::NodeBase {
public:
    Node(uint8_t local_id, const char* instance_name);

    // ─── TX Topic: rpc X(..) returns (stream Msg) ───
    /// 送信バッファへの mutable 参照
    <TopicMsg>& <topic>_buffer();

    /// PDO TX を即時発火。Device::poll() の定期送信とは独立して動作。
    /// 対応する PdoTxEntry が未登録の場合は Status::NOT_FOUND を返す。
    Status publish_<topic>();

    // ─── RX Topic: rpc X(stream Msg) returns (..) ───
    using <Topic>Callback = void(*)(const <TopicMsg>&, void*);
    void on_<topic>(<Topic>Callback cb, void* ctx = nullptr);

    // ─── Service: rpc X(Req) returns (Res) (is_parameter=false) ───
    using <Svc>Callback = Status(*)(const <ReqMsg>&, <ResMsg>&, void*);
    void on_<svc>(<Svc>Callback cb, void* ctx = nullptr);

    // ─── R/W Parameter: rpc X(Req) returns (Res) (is_parameter=true) ───
    using <Param>GetCallback = Status(*)(uint8_t* out, uint8_t& size, void*);
    using <Param>SetCallback = Status(*)(const uint8_t* data, uint8_t size, void*);
    void on_get_<param>(<Param>GetCallback cb, void* ctx = nullptr);
    void on_set_<param>(<Param>SetCallback cb, void* ctx = nullptr);

    // ─── R/O Parameter: rpc X(Empty) returns (Res) (is_parameter=true, req=Empty) ───
    using <Param>GetCallback = Status(*)(uint8_t* out, uint8_t& size, void*);
    void on_get_<param>(<Param>GetCallback cb, void* ctx = nullptr);
    // on_set_<param> は生成しない

    // ─── NodeBase オーバーライド (impl in .cpp) ───
    void    on_pdo_rx(uint16_t pdo_id, const uint8_t* data, uint8_t len) override;
    uint8_t fill_pdo_tx(uint16_t pdo_id, uint8_t* buf, uint8_t max_len) override;
    Status  on_param_read(uint8_t idx, uint8_t* out, uint8_t& out_size) override;
    Status  on_param_write(uint8_t idx, const uint8_t* data, uint8_t sz) override;
    Status  on_service_req(uint8_t svc, const uint8_t* req, uint8_t rsz,
                           uint8_t* res, uint8_t& osz) override;

    // ─── イベント駆動 publish のための送信関数登録 (Device が add_node 時に設定) ───
    using SendPdoFn = Status(*)(uint16_t pdo_id, const uint8_t* data, uint8_t len, void*);
    void set_send_pdo_fn(SendPdoFn fn, void* ctx);

private:
    // TX バッファ (TX Topic 数分)
    <TopicMsg> <topic>_buf_{};

    // RX バッファ (RX Topic 数分)
    <TopicMsg> <topic>_rx_buf_{};

    // コールバックポインタ (各 Topic/Service/Param 分)
    <Topic>Callback  <topic>_cb_  = nullptr; void* <topic>_ctx_  = nullptr;
    <Svc>Callback    <svc>_cb_    = nullptr; void* <svc>_ctx_    = nullptr;
    <Param>GetCallback <param>_get_cb_ = nullptr; void* <param>_get_ctx_ = nullptr;
    <Param>SetCallback <param>_set_cb_ = nullptr; void* <param>_set_ctx_ = nullptr;

    // イベント駆動 publish 用
    SendPdoFn send_pdo_fn_ = nullptr;
    void*     send_pdo_ctx_ = nullptr;
};
```

#### 2.1 トピックインデックスの割り当て

`topic_index` は **TX / RX 別に 0 起算** で `service` 内の rpc 定義順に割り当てる。

```
service BLDCMotor {
  rpc Status(..) returns (stream MotorStatus) {}   → TX topic_index = 0
  rpc Command(stream TwistCommand) returns (..) {} → RX topic_index = 0
  rpc SetEnable(..) returns (..) {}                → Service index = 0
}
```

#### 2.2 `on_pdo_rx` の実装ロジック

```cpp
void Node::on_pdo_rx(uint16_t pdo_id, const uint8_t* data, uint8_t len) {
    for (uint8_t i = 0; i < pdo_rx_count(); ++i) {
        const auto& e = pdo_rx_at(i);
        if (e.pdo_id == pdo_id) {
            switch (e.topic_index) {
                case 0:  // "command" topic (RX index 0)
                    if (e.offset + e.size <= len) {
                        <TopicMsg>_rx_buf_ = <TopicMsg>::decode(data + e.offset);
                        if (<topic>_cb_) <topic>_cb_(<topic>_rx_buf_, <topic>_ctx_);
                    }
                    break;
                // ... 他の RX Topic
            }
        }
    }
}
```

#### 2.3 `fill_pdo_tx` の実装ロジック

```cpp
uint8_t Node::fill_pdo_tx(uint16_t pdo_id, uint8_t* buf, uint8_t max_len) {
    for (uint8_t i = 0; i < pdo_tx_count(); ++i) {
        const auto& e = pdo_tx_at(i);
        if (e.pdo_id == pdo_id) {
            switch (e.topic_index) {
                case 0:  // "status" topic (TX index 0)
                    if (<TopicMsg>::PACKED_SIZE <= max_len) {
                        <topic>_buf_.encode(buf);
                        return static_cast<uint8_t>(<TopicMsg>::PACKED_SIZE);
                    }
                    return 0;
                // ... 他の TX Topic
            }
        }
    }
    return 0;
}
```

#### 2.4 `publish_<topic>` の実装ロジック

```cpp
Status Node::publish_<topic>() {
    if (!send_pdo_fn_) return Status::NOT_FOUND;
    for (uint8_t i = 0; i < pdo_tx_count(); ++i) {
        const auto& e = pdo_tx_at(i);
        if (e.topic_index == <TX_INDEX>) {
            uint8_t buf[<TopicMsg>::PACKED_SIZE];
            <topic>_buf_.encode(buf);
            return send_pdo_fn_(e.pdo_id, buf, sizeof(buf), send_pdo_ctx_);
        }
    }
    return Status::NOT_FOUND;
}
```

#### 2.5 `on_param_read` / `on_param_write` の実装ロジック

`param_index` は `service` 内で `is_parameter=true` の rpc を定義順に 0 起算で割り当て。

```cpp
Status Node::on_param_read(uint8_t idx, uint8_t* out, uint8_t& out_size) {
    switch (idx) {
        case 0:  // "pid_gains" parameter
            if (<param>_get_cb_) return <param>_get_cb_(out, out_size, <param>_get_ctx_);
            return Status::NOT_FOUND;
        // ...
        default: return Status::NOT_FOUND;
    }
}
```

#### 2.6 `on_service_req` の実装ロジック

`service_index` は `service` 内で `is_parameter=false` の unary rpc を定義順に 0 起算で割り当て。

```cpp
Status Node::on_service_req(uint8_t svc, const uint8_t* req, uint8_t rsz,
                            uint8_t* res, uint8_t& osz) {
    switch (svc) {
        case 0: {  // "set_enable" service
            if (!<svc>_cb_) return Status::NOT_FOUND;
            if (rsz < <ReqMsg>::PACKED_SIZE) return Status::INVALID_ARGUMENT;
            auto req_msg = <ReqMsg>::decode(req);
            <ResMsg> res_msg{};
            Status s = <svc>_cb_(req_msg, res_msg, <svc>_ctx_);
            if (s == Status::OK && <ResMsg>::PACKED_SIZE <= 62) {
                res_msg.encode(res);
                osz = static_cast<uint8_t>(<ResMsg>::PACKED_SIZE);
            }
            return s;
        }
        default: return Status::NOT_FOUND;
    }
}
```

---

### R3 — スキーマ定数

```cpp
// <snake_name>.hpp
namespace protocan::<package> {

    constexpr uint32_t SCHEMA_HASH = 0x<FNV1a計算結果>;

    extern const uint8_t  DESCRIPTOR_BLOB[];
    extern const size_t   DESCRIPTOR_BLOB_SIZE;

    constexpr uint8_t MAX_PDO_TX_ENTRIES = <TX Topic 数>;
    constexpr uint8_t MAX_PDO_RX_ENTRIES = <RX Topic 数>;

} // namespace protocan::<package>

// <snake_name>.cpp
const uint8_t protocan::<package>::DESCRIPTOR_BLOB[] = { 0x.., 0x.., ... };
const size_t  protocan::<package>::DESCRIPTOR_BLOB_SIZE = sizeof(DESCRIPTOR_BLOB);
```

`SCHEMA_HASH` は `constexpr` としてヘッダに記述し、コンパイル時に定数として使用可能にする。

---

### R4 — Descriptor Blob 生成手順

コードジェネレータが `.cpp` ファイルに出力する `DESCRIPTOR_BLOB[]` の生成アルゴリズム:

```
1. .proto の service 定義から protocan::NodeDescriptor メッセージを構築する
   ─ schema_hash フィールドは 0 に設定
   ─ node_type_name = "<package>.<ServiceName>"
   ─ ros2_namespace = protocan.node.ros2_namespace (未指定時は snake_case 変換)
   ─ topics[]: TX rpc → is_tx=true, RX rpc → is_tx=false
               index はそれぞれ 0 起算
               message.payload_size, fields[].offset/size を計算して設定
   ─ services[]: is_parameter=false の unary rpc。index は 0 起算
   ─ params[]: is_parameter=true の rpc。index は 0 起算

2. io::CodedOutputStream::SetSerializationDeterministic(true) を設定し、
   NodeDescriptor を Protobuf deterministic serialization でシリアライズする

3. シリアライズ結果バイト列に FNV-1a (32-bit) ハッシュを計算する
   ─ FNV offset basis: 0x811c9dc5
   ─ FNV prime:        0x01000193
   ─ アルゴリズム:
       hash = FNV_OFFSET_BASIS
       for each byte b:
           hash ^= b
           hash *= FNV_PRIME
           hash &= 0xFFFFFFFF

4. NodeDescriptor.schema_hash フィールドに step 3 の値を設定し、再度
   deterministic serialization する → これが最終 Descriptor Blob

5. Blob を C 配列リテラルとして .cpp に出力する
   例: const uint8_t DESCRIPTOR_BLOB[] = { 0x0A, 0x14, ... };
```

---

### R5 — 名前空間と命名規則

#### 名前空間

```
package bldc_motor;        → namespace protocan::bldc_motor {}
package imu_sensor;        → namespace protocan::imu_sensor {}
(package 未指定)            → namespace protocan {}
```

生成コードはすべて `protocan::<package>` 名前空間に属する。

#### ファイル名

`service` 名を CamelCase → snake_case に変換してファイル名とする。

```
service BLDCMotor {}    → bldc_motor.hpp / bldc_motor.cpp
service IMUSensor {}    → imu_sensor.hpp / imu_sensor.cpp
```

変換規則:
1. 大文字の連続 (例: `BLDC`) はそのまま小文字化して `bldc` とする
2. 大文字 → `_` + 小文字 に変換 (ただし先頭の大文字はアンダースコアなし)
3. 連続アンダースコアは 1 つに圧縮する

#### メンバ名

| 対象 | 命名規則 | 例 |
|------|---------|-----|
| TX Topic バッファ | `<rpc_name>_buffer()` | `status_buffer()` |
| TX Topic 発火 | `publish_<rpc_name>()` | `publish_status()` |
| RX Topic コールバック登録 | `on_<rpc_name>(cb, ctx)` | `on_command(cb, ctx)` |
| Service コールバック登録 | `on_<rpc_name>(cb, ctx)` | `on_set_enable(cb, ctx)` |
| Parameter GET | `on_get_<rpc_name>(cb, ctx)` | `on_get_pid_gains(cb, ctx)` |
| Parameter SET | `on_set_<rpc_name>(cb, ctx)` | `on_set_pid_gains(cb, ctx)` |
| using 型 | `<RpcName>Callback` | `StatusCallback` |
| private バッファ | `<rpc_name>_buf_` | `status_buf_` |
| private RX バッファ | `<rpc_name>_rx_buf_` | `command_rx_buf_` |
| private コールバック | `<rpc_name>_cb_` | `command_cb_` |
| private コンテキスト | `<rpc_name>_ctx_` | `command_ctx_` |

rpc_name は CamelCase → snake_case 変換後の名前。`ros2_name` オプションが指定された場合はそちらを優先する。

---

### R6 — 生成コード一般要件

| 項目 | 要件 |
|------|------|
| C++ 標準 | C++17 以上 |
| ヒープ使用 | 禁止。`std::vector`, `std::function`, `new` / `delete` を使用しない |
| `constexpr` | `SCHEMA_HASH`, `PACKED_SIZE`, `MAX_PDO_TX_ENTRIES`, `MAX_PDO_RX_ENTRIES` |
| コールバック | 関数ポインタ + `void*` コンテキスト (ラムダ・std::function 不使用) |
| `#pragma once` | 生成するすべてのヘッダに付与する |
| encode/decode | `memcpy` ベースの LE エンコード。型パンニング UB を避けるため `float` / `double` も `memcpy` 経由 |
| ゼロ初期化 | コンストラクタで全バッファをゼロ初期化する (`= {}`) |
| インクルード | `#include "protocan_device/node_base.hpp"` のみ (Protobuf ヘッダは不要) |
| コピー | `Node` クラスはコピー禁止 (`= delete`) にする。デバイス上で重複インスタンスを防ぐため |

---

## 5. 生成コード全体例

`bldc_motor.proto` の `BLDCMotor` service に対して生成されるコードの例。

### bldc_motor.hpp (抜粋)

```cpp
#pragma once
#include <cstdint>
#include <cstring>
#include "protocan_device/node_base.hpp"

namespace protocan::bldc_motor {

// ─── スキーマ定数 ───
constexpr uint32_t SCHEMA_HASH        = 0xA1B2C3D4u;
constexpr uint8_t  MAX_PDO_TX_ENTRIES = 1;  // Status
constexpr uint8_t  MAX_PDO_RX_ENTRIES = 1;  // Command
extern const uint8_t DESCRIPTOR_BLOB[];
extern const size_t  DESCRIPTOR_BLOB_SIZE;

// ─── メッセージ構造体 ───
struct MotorStatus {
    float   current_a;
    float   velocity_rps;
    float   temperature_c;
    uint8_t error_flags;   // proto: uint32, size=1 → uint8_t

    static constexpr size_t PACKED_SIZE = 13;
    void encode(uint8_t* buf) const;
    static MotorStatus decode(const uint8_t* buf);
};

struct TwistCommand {
    float linear_x;
    float angular_z;

    static constexpr size_t PACKED_SIZE = 8;
    void encode(uint8_t* buf) const;
    static TwistCommand decode(const uint8_t* buf);
};

struct SetEnableRequest {
    bool enable;
    static constexpr size_t PACKED_SIZE = 1;
    void encode(uint8_t* buf) const;
    static SetEnableRequest decode(const uint8_t* buf);
};

struct SetEnableResponse {
    bool success;
    static constexpr size_t PACKED_SIZE = 1;
    void encode(uint8_t* buf) const;
    static SetEnableResponse decode(const uint8_t* buf);
};

// ─── Node クラス ───
class Node : public protocan::device::NodeBase {
public:
    Node(uint8_t local_id, const char* instance_name);
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    // TX Topic: Status
    MotorStatus& status_buffer();
    Status publish_status();

    // RX Topic: Command (ros2_name="cmd_vel")
    using CommandCallback = void(*)(const TwistCommand&, void*);
    void on_command(CommandCallback cb, void* ctx = nullptr);

    // Service: SetEnable
    using SetEnableCallback = Status(*)(const SetEnableRequest&, SetEnableResponse&, void*);
    void on_set_enable(SetEnableCallback cb, void* ctx = nullptr);

    // NodeBase overrides
    void    on_pdo_rx(uint16_t pdo_id, const uint8_t* data, uint8_t len) override;
    uint8_t fill_pdo_tx(uint16_t pdo_id, uint8_t* buf, uint8_t max_len) override;
    Status  on_param_read(uint8_t idx, uint8_t* out, uint8_t& out_size) override;
    Status  on_param_write(uint8_t idx, const uint8_t* data, uint8_t sz) override;
    Status  on_service_req(uint8_t svc, const uint8_t* req, uint8_t rsz,
                           uint8_t* res, uint8_t& osz) override;

    using SendPdoFn = Status(*)(uint16_t pdo_id, const uint8_t* data, uint8_t len, void*);
    void set_send_pdo_fn(SendPdoFn fn, void* ctx);

private:
    MotorStatus  status_buf_{};
    TwistCommand command_rx_buf_{};

    CommandCallback  command_cb_    = nullptr; void* command_ctx_    = nullptr;
    SetEnableCallback set_enable_cb_ = nullptr; void* set_enable_ctx_ = nullptr;

    SendPdoFn send_pdo_fn_  = nullptr;
    void*     send_pdo_ctx_ = nullptr;
};

} // namespace protocan::bldc_motor
```

### bldc_motor.cpp (抜粋)

```cpp
#include "bldc_motor.hpp"
#include <cstring>

namespace protocan::bldc_motor {

// ─── Descriptor Blob ───
const uint8_t DESCRIPTOR_BLOB[] = { 0x0A, 0x14, /* ... FNV-1a込みのシリアライズ結果 */ };
const size_t  DESCRIPTOR_BLOB_SIZE = sizeof(DESCRIPTOR_BLOB);

// ─── MotorStatus encode/decode ───
void MotorStatus::encode(uint8_t* buf) const {
    uint32_t tmp;
    std::memcpy(&tmp, &current_a,    4); buf[0]=tmp; buf[1]=tmp>>8; buf[2]=tmp>>16; buf[3]=tmp>>24;
    std::memcpy(&tmp, &velocity_rps, 4); buf[4]=tmp; buf[5]=tmp>>8; buf[6]=tmp>>16; buf[7]=tmp>>24;
    std::memcpy(&tmp, &temperature_c,4); buf[8]=tmp; buf[9]=tmp>>8; buf[10]=tmp>>16; buf[11]=tmp>>24;
    buf[12] = static_cast<uint8_t>(error_flags);
}
MotorStatus MotorStatus::decode(const uint8_t* buf) {
    MotorStatus m{};
    uint32_t tmp;
    tmp = uint32_t(buf[0])|(uint32_t(buf[1])<<8)|(uint32_t(buf[2])<<16)|(uint32_t(buf[3])<<24);
    std::memcpy(&m.current_a, &tmp, 4);
    // ... 同様に残フィールド
    m.error_flags = buf[12];
    return m;
}

// ─── Node ───
Node::Node(uint8_t local_id, const char* instance_name)
    : NodeBase(local_id, instance_name, DESCRIPTOR_BLOB, DESCRIPTOR_BLOB_SIZE, SCHEMA_HASH)
{}

MotorStatus& Node::status_buffer() { return status_buf_; }

Status Node::publish_status() {
    if (!send_pdo_fn_) return Status::NOT_FOUND;
    for (uint8_t i = 0; i < pdo_tx_count(); ++i) {
        if (pdo_tx_at(i).topic_index == 0) {
            uint8_t buf[MotorStatus::PACKED_SIZE];
            status_buf_.encode(buf);
            return send_pdo_fn_(pdo_tx_at(i).pdo_id, buf, sizeof(buf), send_pdo_ctx_);
        }
    }
    return Status::NOT_FOUND;
}

void Node::on_command(CommandCallback cb, void* ctx) {
    command_cb_  = cb;
    command_ctx_ = ctx;
}
void Node::on_set_enable(SetEnableCallback cb, void* ctx) {
    set_enable_cb_  = cb;
    set_enable_ctx_ = ctx;
}

void Node::on_pdo_rx(uint16_t pdo_id, const uint8_t* data, uint8_t len) {
    for (uint8_t i = 0; i < pdo_rx_count(); ++i) {
        const auto& e = pdo_rx_at(i);
        if (e.pdo_id == pdo_id) {
            if (e.topic_index == 0) {  // command
                if (e.offset + TwistCommand::PACKED_SIZE <= len) {
                    command_rx_buf_ = TwistCommand::decode(data + e.offset);
                    if (command_cb_) command_cb_(command_rx_buf_, command_ctx_);
                }
            }
        }
    }
}

uint8_t Node::fill_pdo_tx(uint16_t pdo_id, uint8_t* buf, uint8_t max_len) {
    for (uint8_t i = 0; i < pdo_tx_count(); ++i) {
        if (pdo_tx_at(i).pdo_id == pdo_id) {
            if (pdo_tx_at(i).topic_index == 0 && MotorStatus::PACKED_SIZE <= max_len) {
                status_buf_.encode(buf);
                return static_cast<uint8_t>(MotorStatus::PACKED_SIZE);
            }
        }
    }
    return 0;
}

// on_param_read / on_param_write → BLDCMotor に Parameter がない場合
Status Node::on_param_read(uint8_t, uint8_t*, uint8_t&)        { return Status::NOT_FOUND; }
Status Node::on_param_write(uint8_t, const uint8_t*, uint8_t)  { return Status::NOT_FOUND; }

Status Node::on_service_req(uint8_t svc, const uint8_t* req, uint8_t rsz,
                            uint8_t* res, uint8_t& osz) {
    switch (svc) {
        case 0: {  // set_enable
            if (!set_enable_cb_) return Status::NOT_FOUND;
            if (rsz < SetEnableRequest::PACKED_SIZE) return Status::INVALID_ARGUMENT;
            auto req_msg = SetEnableRequest::decode(req);
            SetEnableResponse res_msg{};
            Status s = set_enable_cb_(req_msg, res_msg, set_enable_ctx_);
            if (s == Status::OK) { res_msg.encode(res); osz = SetEnableResponse::PACKED_SIZE; }
            return s;
        }
        default: return Status::NOT_FOUND;
    }
}

void Node::set_send_pdo_fn(SendPdoFn fn, void* ctx) {
    send_pdo_fn_  = fn;
    send_pdo_ctx_ = ctx;
}

} // namespace protocan::bldc_motor
```

---

## 6. Device::add_node の拡張

`Device::add_node()` は Node 登録時に `set_send_pdo_fn` を呼び出して
イベント駆動 publish 用の送信関数を注入する。

```cpp
// protocan_device/src/device.cpp

static Status device_send_pdo(uint16_t pdo_id, const uint8_t* data, uint8_t len, void* ctx) {
    auto* self = static_cast<Device*>(ctx);
    CanFrame f = make_standard_frame(pdo_id, data, len);
    return self->send_frame_pub(f);  // または既存の send_frame を公開
}

Status Device::add_node(NodeBase& node) {
    if (node_count_ >= kMaxNodes) return Status::NO_RESOURCE;
    nodes_[node_count_++] = &node;

    // イベント駆動 publish 用コールバックを注入
    // NodeBase に set_send_pdo_fn が存在する場合のみ (dynamic_cast を避けるため仮想化)
    node.set_send_pdo(device_send_pdo, this);

    return Status::OK;
}
```

> **Note**: `NodeBase` にも `virtual void set_send_pdo(SendPdoFn fn, void* ctx) {}` をデフォルト実装として追加し、生成コードが override する形が望ましい。

---

## 7. ジェネレータの実装方針

### 7.1 実装言語

C++ (`protoc` プラグイン API を使用)。または Go / Python による protoc プラグインでも可。

### 7.2 protoc プラグインとしての起動

```sh
protoc \
  --proto_path=proto \
  --protocan_out=gen \
  --plugin=protoc-gen-protocan=./bin/protoc-gen-protocan \
  proto/bldc_motor.proto
```

### 7.3 実装ステップ

1. `google::protobuf::compiler::CodeGenerator` を継承したクラスを実装する
2. `ServiceDescriptorProto` を走査して RPC パターンを分類する
3. `FieldDescriptorProto` からフィールドのオフセットと型を計算する
4. §4 の手順で `NodeDescriptor` を構築し、FNV-1a ハッシュを計算する
5. Descriptor Blob をシリアライズして C 配列リテラルに変換する
6. テンプレートエンジン（または文字列連結）で `.hpp` / `.cpp` を出力する

### 7.4 エラーチェック

生成時に以下をチェックし、エラーメッセージを出力してコード生成を中断する:

| 条件 | エラーメッセージ例 |
|------|--------------------|
| サポート外フィールド型 (`string`, `bytes`, `repeated` 等) | `"Field X: type Y is not supported by protoc-gen-protocan"` |
| `protocan.field.size` に 1/2 以外の値 | `"Field X: size must be 1 or 2"` |
| TX Topic メッセージが 64 バイトを超える | `"Topic X: PACKED_SIZE exceeds CAN FD payload (64 bytes)"` |
| Service Req/Res メッセージが 62 バイトを超える (警告のみ) | `"Service X: response exceeds 62 bytes, BULK transfer will be used"` |
| bidirectional streaming rpc (`stream Req → stream Res`) | `"RPC X: bidirectional streaming is not supported"` |

---

## 8. ビルド統合

### CMake での組み込み例

```cmake
# 生成コマンドをカスタムターゲットとして定義
find_program(PROTOC_GEN_PROTOCAN protoc-gen-protocan REQUIRED)

function(protocan_generate TARGET PROTO_FILE)
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
    set(GEN_HPP "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.hpp")
    set(GEN_CPP "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.cpp")

    add_custom_command(
        OUTPUT ${GEN_HPP} ${GEN_CPP}
        COMMAND protoc
            --proto_path=${CMAKE_SOURCE_DIR}/proto
            --protocan_out=${CMAKE_CURRENT_BINARY_DIR}
            --plugin=protoc-gen-protocan=${PROTOC_GEN_PROTOCAN}
            ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}
        COMMENT "Generating device C++ from ${PROTO_FILE}"
    )

    target_sources(${TARGET} PRIVATE ${GEN_CPP})
    target_include_directories(${TARGET} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
endfunction()

# 使用例
add_executable(firmware main.cpp)
target_link_libraries(firmware protocan_device)
protocan_generate(firmware proto/bldc_motor.proto)
```

---

## 9. 用語集

| 用語 | 定義 |
|------|------|
| TX Topic | デバイスが `publish` するデータストリーム。`returns (stream Msg)` パターン |
| RX Topic | デバイスが `subscribe` するデータストリーム。`(stream Msg) returns` パターン |
| topic_index | TX または RX それぞれ独立した 0 起算の整数インデックス |
| service_index | `is_parameter=false` の unary rpc に付与される 0 起算のインデックス |
| param_index | `is_parameter=true` の rpc に付与される 0 起算のインデックス |
| Packed Binary | 固定長・フィールド番号順・LE エンコードの CAN ペイロードバイナリ形式 |
| PACKED_SIZE | 1 メッセージを Packed Binary でエンコードしたバイト数 (`constexpr` 定数) |
| Descriptor Blob | `protocan::NodeDescriptor` の Protobuf deterministic シリアライズ結果 |
| SCHEMA_HASH | Descriptor Blob の FNV-1a (32-bit) ハッシュ値。`constexpr uint32_t` |
| send_pdo_fn | Node が Device から受け取るイベント駆動 PDO 送信用関数ポインタ |
