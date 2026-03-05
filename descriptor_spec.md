# ProtoCAN — Descriptor Blob 仕様

## 1. 概要

本ドキュメントは、ProtoCAN 対応デバイスが内蔵する「Descriptor Blob（ディスクリプタバイナリ）」のデータ構造とエンコーディングを定義する。

ProtoCAN では、Descriptor Blob は **デバイス全体ではなく、Node Type（`.proto` の `service` 定義）単位** で生成・管理される。
デバイスはこの Blob を不透明なバイト配列として定数 (ROM) に持ち、マスターからの DISC (0x2) 要求に対して BULK 転送で返答する。
マスター（ブリッジ）側はこのバイナリをデコードすることで、ROS 2 ノード名、メッセージ型、フィールド構造、パラメータ、サービスなどの構成を動的に復元する。

## 2. エンコーディング

Descriptor Blob は **Protocol Buffers (proto3)** でエンコードされる。メッセージ定義は [`protocan/descriptor.proto`](file:///home/f0reach/workspace/protocan/protocan/descriptor.proto) に集約される。

### 2.1 設計方針

- **Protobuf エンコーディング**: Descriptor Blob は `protocan.NodeDescriptor` メッセージを Protocol Buffers の標準ワイヤフォーマットでシリアライズしたバイト列である。
- **Deterministic Serialization**: Schema Hash 計算の再現性を保証するため、シリアライズは **決定論的 (deterministic)** に行う。具体的には:
  - `repeated` フィールドは定義順（インデックス順）に出力する
  - Map フィールドは使用しない
  - C++ では `SerializeToString` に `SetSerializationDeterministic(true)` を設定する
- **拡張性**: Protobuf のフィールド追加による後方互換性を活用し、将来の仕様拡張に対応する。
- **コンパクト性**: Protobuf の可変長エンコーディング (varint) により、小さな値は少ないバイト数で表現される。

### 2.2 旧 Packed Binary 形式との違い

| 観点 | 旧 (Packed Binary) | 新 (Protobuf) |
|------|---------------------|---------------|
| エンコーディング | 独自固定長バイナリ | Protobuf wire format |
| Magic Number / Version | ヘッダに含む | 不要（Protobuf 自体が自己記述的） |
| 文字列 | Null 終端 C-string | Protobuf の length-delimited string |
| フィールド情報 | なし | `FieldDescriptor` で型・オフセット・ROS 2 マッピングを格納 |
| 拡張性 | Version bump が必要 | フィールド追加で後方互換 |

## 3. メッセージ定義

ルート型は `protocan.NodeDescriptor` であり、`.proto` の `service` 定義 1 つにつき 1 つの `NodeDescriptor` が生成される。

### 3.1 全体構造

```
NodeDescriptor
├── schema_hash     : uint32 (FNV-1a)
├── node_type_name  : string (例: "bldc_motor.BLDCMotor")
├── ros2_namespace  : string (省略時は空)
├── topics[]        : TopicDescriptor
│   ├── index, name, is_tx, periodic, priority
│   └── message     : MessageDescriptor
│       ├── ros2_msg_type, payload_size
│       └── fields[] : FieldDescriptor
│           ├── name, type (FieldType enum), offset, size
│           └── ros2_field (ROS 2 フィールドパス)
├── services[]      : ServiceDescriptor
│   ├── index, name, ros2_srv_type
│   ├── request     : MessageDescriptor (→ fields[])
│   └── response    : MessageDescriptor (→ fields[])
└── params[]        : ParamDescriptor
    ├── index, name, type (FieldType enum)
    └── read_only
```

### 3.2 FieldType enum

`FieldType` は Packed Binary でのバイト表現を示す。Proto 型との対応は以下の通り:

| FieldType | Proto 型 | CAN バイナリ | サイズ |
|-----------|----------|-------------|-------|
| `FIELD_TYPE_BOOL`   | `bool`   | uint8 (0/1) | 1B |
| `FIELD_TYPE_UINT8`  | *(カスタム拡張)* | LE u8  | 1B |
| `FIELD_TYPE_INT8`   | *(カスタム拡張)* | LE i8  | 1B |
| `FIELD_TYPE_UINT16` | *(カスタム拡張)* | LE u16 | 2B |
| `FIELD_TYPE_INT16`  | *(カスタム拡張)* | LE i16 | 2B |
| `FIELD_TYPE_UINT32` | `uint32` | LE u32 | 4B |
| `FIELD_TYPE_INT32`  | `int32`  | LE i32 | 4B |
| `FIELD_TYPE_FLOAT`  | `float`  | IEEE 754 LE | 4B |
| `FIELD_TYPE_DOUBLE` | `double` | IEEE 754 LE | 8B |
| `FIELD_TYPE_UINT64` | `uint64` | LE u64 | 8B |
| `FIELD_TYPE_INT64`  | `int64`  | LE i64 | 8B |

> **Note**: `uint8` / `int8` / `uint16` / `int16` は Proto 型としてネイティブに存在しない。`protoc-gen-protocan` がカスタムオプション等で指示されたサイズ情報に基づき適切な `FieldType` を選択する。Proto ワイヤ上では `uint32` / `int32` として扱われるが、CAN バイナリ (Packed Binary) では指定されたサイズでエンコードされる。

### 3.3 FieldDescriptor

| フィールド | 型 | 説明 |
|------------|-----|------|
| `name` | string | proto フィールド名 (例: `"current_a"`) |
| `type` | FieldType | CAN バイナリ型 |
| `offset` | uint32 | Packed Binary 内のバイトオフセット |
| `size` | uint32 | バイトサイズ |
| `ros2_field` | string | ROS 2 メッセージ内のフィールドパス (例: `"linear.x"`)。`ros2_msg_type` 指定時のマッピングに使用。未指定時は空。 |

### 3.4 MessageDescriptor

| フィールド | 型 | 説明 |
|------------|-----|------|
| `ros2_msg_type` | string | 対応する ROS 2 メッセージ型 (例: `"geometry_msgs/msg/Twist"`)。省略時はブリッジが動的メッセージを構築。 |
| `payload_size` | uint32 | Packed Binary 全体のバイトサイズ |
| `fields` | FieldDescriptor[] | フィールド定義（proto フィールド番号順） |

### 3.5 デバイス間ルーティングにおけるフィールド互換性検証

マスターはデバイス間直接ルーティング (spec.md §6.1) の設定時に、送信側・受信側双方の `MessageDescriptor` から以下を比較してバイト列レベルの互換性を検証する:

1. `payload_size` が一致していること
2. `fields` の数が一致し、各フィールドの `type`, `offset`, `size` が全て一致していること

フィールド名 (`name`) や ROS 2 マッピング (`ros2_field`) の一致は要求しない。

## 4. Schema Hash (schema_hash) の計算

`schema_hash` は、`protoc-gen-protocan` がコンパイル時に以下の手順で計算する:

1. `NodeDescriptor` メッセージを構築する（`schema_hash` フィールドは `0` に設定）
2. **Deterministic serialization** でバイト列にシリアライズする
3. シリアライズ結果に対して **FNV-1a (32-bit)** ハッシュを計算する
4. 得られたハッシュ値を `schema_hash` フィールドに格納して、最終的な Descriptor Blob を生成する

同じ `.proto` 定義から生成される `NodeDescriptor` は、Protobuf の deterministic serialization により常に同一のバイト列を生成するため、環境依存によるハッシュ揺らぎは発生しない。

## 5. 生成例

`bldc_motor.proto` の `BLDCMotor` service から生成される `NodeDescriptor` の概念的な内容:

```
NodeDescriptor {
  schema_hash: 0xA1B2C3D4
  node_type_name: "bldc_motor.BLDCMotor"
  ros2_namespace: "motor"
  topics: [
    TopicDescriptor {
      index: 0
      name: "status"
      is_tx: true
      periodic: true
      priority: 3
      message: MessageDescriptor {
        payload_size: 13
        fields: [
          { name: "current_a",     type: FLOAT,  offset: 0,  size: 4 }
          { name: "velocity_rps",  type: FLOAT,  offset: 4,  size: 4 }
          { name: "temperature_c", type: FLOAT,  offset: 8,  size: 4 }
          { name: "error_flags",   type: UINT32, offset: 12, size: 4 }
        ]
      }
    }
    TopicDescriptor {
      index: 1
      name: "cmd_vel"
      is_tx: false
      message: MessageDescriptor {
        ros2_msg_type: "geometry_msgs/msg/Twist"
        payload_size: 8
        fields: [
          { name: "linear_x",  type: FLOAT, offset: 0, size: 4, ros2_field: "linear.x" }
          { name: "angular_z", type: FLOAT, offset: 4, size: 4, ros2_field: "angular.z" }
        ]
      }
    }
  ]
  services: [
    ServiceDescriptor {
      index: 0
      name: "set_enable"
      ros2_srv_type: "std_srvs/srv/SetBool"
      request: MessageDescriptor {
        payload_size: 1
        fields: [
          { name: "enable", type: BOOL, offset: 0, size: 1, ros2_field: "data" }
        ]
      }
      response: MessageDescriptor {
        payload_size: 1
        fields: [
          { name: "success", type: BOOL, offset: 0, size: 1, ros2_field: "success" }
        ]
      }
    }
  ]
}
```
