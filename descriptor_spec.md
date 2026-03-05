# ProtoCAN — Descriptor Blob 仕様

## 1. 概要

本ドキュメントは、ProtoCAN 対応デバイスが内蔵する「Descriptor Blob（ディスクリプタバイナリ）」のデータ構造レイアウトを定義する。
デバイスはこの Blob を不透明なバイト配列として定数 (ROM) に持ち、マスターからの DISC (0x2) 要求に対してそのまま BULK 転送で返答する。
マスター（ブリッジ）側はこのバイナリをパースすることで、ROS 2 ノード名、メッセージ型、パラメータ、サービスなどの構成を動的に復元する。

## 2. 設計方針

- **リトルエンディアン**: すべてのマルチバイト整数はリトルエンディアン (LE) でエンコードされる。
- **コンパクト性**: アライメントパディングは含めず、隙間なくパッキングする (Packed Binary)。
- **拡張性**: 構造変更に備え、ヘッダにフォーマットバージョンを含める。
- **文字列**: 全ての文字列（ノード名、ROS 2 メッセージ型名など）は `Null終端文字列 (C-string)` として格納される。

## 3. 全体レイアウト

Descriptor Blob は「ヘッダ (Header)」と、それに続く複数の「ノードディスクリプタ (Node Descriptor)」から構成される。

| オフセット | サイズ | フィールド名 | 説明 |
|------------|--------|--------------|------|
| 0x00 | 4 | Magic Number | `0x50, 0x43, 0x41, 0x4E` ("PCAN") |
| 0x04 | 1 | Version | ディスクリプタのバージョン (現在は `0x01`) |
| 0x05 | 2 | Total Size | ヘッダを含む Descriptor Blob 全体のバイト数 (uint16_t) |
| 0x07 | 1 | Num Nodes | 含まれる Node Descriptor の数 |
| 0x08... | 可変 | Node Descriptors| Node Descriptor の配列 (Num Nodes 回繰り返す) |

## 4. Node Descriptor

各ノード（例: `.proto` での `service` に相当）の定義。

| サイズ | フィールド名 | 説明 |
|--------|--------------|------|
| 1 | Local Node ID | デバイス内でのノードインデックス (0~15) |
| 可変 | Node Type Name | Null終端文字列 (例: "bldc_motor.BLDCMotor") |
| 可変 | ROS 2 Namespace| Null終端文字列 (省略された場合は空文字列 `\0`) |
| 1 | Num Topics | このノードが持つ Topic 数 |
| 可変 | Topics | Topic Descriptor の配列 |
| 1 | Num Services | このノードが持つ Service 数 |
| 可変 | Services | Service Descriptor の配列 |
| 1 | Num Params | このノードが持つ Parameter 数 |
| 可変 | Params | Parameter Descriptor の配列 |

## 5. Topic Descriptor

| サイズ | フィールド名 | 説明 |
|--------|--------------|------|
| 1 | Topic Index | ノード内での Topic インデックス |
| 1 | Direction | `0x00`: TX (デバイス→マスター), `0x01`: RX (マスター→デバイス) |
| 1 | Flags | `Bit 0`: Periodic (周期送信かどうかのヒント)<br>`Bit 1-3`: Priority (0-7)<br>`Bit 4-7`: Reserved |
| 可変 | Name | RPC名または `ros2_name` オプションで上書きされた名前 (Null終端) |
| 可変 | ROS 2 Msg Type| ROS 2 のメッセージ型名 (例: "geometry_msgs/msg/Twist" / 指定なしは空 `\0`) |
| 2 | Payload Size | このトピックのメッセージが持つ CAN ペイロードサイズ (バイト数) |

## 6. Service Descriptor

| サイズ | フィールド名 | 説明 |
|--------|--------------|------|
| 1 | Service Index | ノード内での Service インデックス |
| 可変 | Name | RPC名または `ros2_name` (Null終端) |
| 可変 | ROS 2 Srv Type| ROS 2 のサービス型名 (例: "std_srvs/srv/SetBool" / 指定なしは空 `\0`) |
| 2 | Req Payload Size| Request のペイロード最大サイズ (バイト数) |
| 2 | Res Payload Size| Response のペイロード最大サイズ (バイト数) |

## 7. Parameter Descriptor

| サイズ | フィールド名 | 説明 |
|--------|--------------|------|
| 1 | Param Index | ノード内での Parameter インデックス |
| 1 | Flags | `Bit 0`: Read-only, `Bit 1-7`: Reserved |
| 1 | Data Type | パラメータの型ID (0=bool, 1=uint32, 2=int32, 3=float, 4=double, etc.) |
| 可変 | Name | パラメータ名 (Null終端) |

## 8. スキーマハッシュ (schema_hash) との関連

`schema_hash` は、コンパイル時に生成されたこの **Descriptor Blob 全体のバイナリデータを FNV-1a (32-bit) ハッシュ関数にかけた値** である。
これにより、プロトコル定義（.proto）やROS 2 マッピングの定義内容が変更されればバイナリ構造が変わり、異なるハッシュ値としてマスターに認識され、変更が自動検知される。
