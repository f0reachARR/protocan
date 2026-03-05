# ProtoCAN — Descriptor Blob 仕様

## 1. 概要

本ドキュメントは、ProtoCAN 対応デバイスが内蔵する「Descriptor Blob（ディスクリプタバイナリ）」のデータ構造レイアウトを定義する。
ProtoCAN では、Descriptor Blob は **デバイス全体ではなく、Node Type（`.proto` の `service` 定義）単位** で生成・管理される。
デバイスはこの Blob を不透明なバイト配列として定数 (ROM) に持ち、マスターからの DISC (0x2) 要求に対して BULK 転送で返答する。
マスター（ブリッジ）側はこのバイナリをパースすることで、ROS 2 ノード名、メッセージ型、パラメータ、サービスなどの構成を動的に復元する。

## 2. 設計方針

- **リトルエンディアン**: すべてのマルチバイト整数はリトルエンディアン (LE) でエンコードされる。
- **コンパクト性**: アライメントパディングは含めず、隙間なくパッキングする (Packed Binary)。
- **拡張性**: 構造変更に備え、ヘッダにフォーマットバージョンを含める。
- **文字列**: 全ての文字列（ノード名、ROS 2 メッセージ型名など）は `Null終端文字列 (C-string)` として格納される。

## 3. 全体レイアウト (Node Type Descriptor)

各 `.proto` ファイル内の `service` 定義ごとに1つの Descriptor Blob が生成される。

| オフセット | サイズ | フィールド名 | 説明 |
|------------|--------|--------------|------|
| 0x00 | 4 | Magic Number | `0x50, 0x4E, 0x44, 0x00` ("PND\0" = ProtoCAN Node Descriptor) |
| 0x04 | 1 | Version | ディスクリプタのバージョン (現在は `0x01`) |
| 0x05 | 2 | Total Size | ヘッダを含む Descriptor Blob 全体のバイト数 (uint16_t) |
| 0x07 | 4 | Schema Hash  | この Blob 自体の内容（Hashフィールドを0とした状態）から計算された FNV-1a (32-bit) ハッシュ値 |
| 0x0B...| 可変| Node Type Name| Node Type 名 (Null終端文字列、例: "bldc_motor.BLDCMotor") |
| 可変 | 可変| ROS 2 Namespace| ROS 2 のデフォルトネームスペース (Null終端文字列、省略時は `\0`) |
| 可変 | 1  | Num Topics   | このノードが持つ Topic 数 |
| 可変 | 可変| Topics       | Topic Descriptor の配列 |
| 可変 | 1  | Num Services | このノードが持つ Service 数 |
| 可変 | 可変| Services     | Service Descriptor の配列 |
| 可変 | 1  | Num Params   | このノードが持つ Parameter 数 |
| 可変 | 可変| Params       | Parameter Descriptor の配列 |

## 4. Topic Descriptor

| サイズ | フィールド名 | 説明 |
|--------|--------------|------|
| 1 | Topic Index | ノード内での Topic インデックス |
| 1 | Direction | `0x00`: TX (デバイス→マスター), `0x01`: RX (マスター→デバイス) |
| 1 | Flags | `Bit 0`: Periodic (周期送信かどうかのヒント)<br>`Bit 1-3`: Priority (0-7)<br>`Bit 4-7`: Reserved |
| 可変 | Name | RPC名または `ros2_name` オプションで上書きされた名前 (Null終端) |
| 可変 | ROS 2 Msg Type| ROS 2 のメッセージ型名 (例: "geometry_msgs/msg/Twist" / 指定なしは空 `\0`) |
| 2 | Payload Size | このトピックのメッセージが持つ CAN ペイロードサイズ (バイト数) |

## 5. Service Descriptor

| サイズ | フィールド名 | 説明 |
|--------|--------------|------|
| 1 | Service Index | ノード内での Service インデックス |
| 可変 | Name | RPC名または `ros2_name` (Null終端) |
| 可変 | ROS 2 Srv Type| ROS 2 のサービス型名 (例: "std_srvs/srv/SetBool" / 指定なしは空 `\0`) |
| 2 | Req Payload Size| Request のペイロード最大サイズ (バイト数) |
| 2 | Res Payload Size| Response のペイロード最大サイズ (バイト数) |

## 6. Parameter Descriptor

| サイズ | フィールド名 | 説明 |
|--------|--------------|------|
| 1 | Param Index | ノード内での Parameter インデックス |
| 1 | Flags | `Bit 0`: Read-only, `Bit 1-7`: Reserved |
| 1 | Data Type | パラメータの型ID (0=bool, 1=uint32, 2=int32, 3=float, 4=double, etc.) |
| 可変 | Name | パラメータ名 (Null終端) |

## 7. スキーマハッシュ (schema_hash) との関連

`schema_hash` は、コンパイル時に生成されたこの **Node Type Descriptor (Descriptor Blob) のバイナリデータ（ハッシュ格納部を0で埋めた状態）を FNV-1a (32-bit) ハッシュ関数にかけた値** である。
プロトコル定義（.proto）やROS 2 マッピングの内容が変更されればバイナリ構造が変わり、異なるハッシュ値としてマスターに認識され、マスターからの再取得（DISC）がトリガされる。
同じ Node Type を持つ複数インスタンス（複数の `local_node_id`）は全く同じハッシュ値を持つため、マスターは最初の1回だけディスクリプタを要求すれば済む設計となっている。
