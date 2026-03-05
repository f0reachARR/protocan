#pragma once

#include <functional>
#include <optional>

#include "protocan/can_frame.hpp"

namespace protocan
{

/// CAN バス HAL 抽象インターフェース
///
/// 具体的な実装 (SocketCAN 等) は protocan-bridge 等の上位パッケージで提供する。
class ICanInterface
{
public:
  virtual ~ICanInterface() = default;

  /// フレームを送信する
  /// @return 送信成功時 Status::OK
  virtual Status send(const CanFrame & frame) = 0;

  /// フレームを受信する（ノンブロッキング）
  /// @return 受信フレームがある場合はそのフレーム、ない場合は std::nullopt
  virtual std::optional<CanFrame> receive() = 0;

  /// CAN バスのオープン/接続
  virtual Status open() = 0;

  /// CAN バスのクローズ/切断
  virtual Status close() = 0;

  /// バスがオープン状態か
  virtual bool is_open() const = 0;
};

}  // namespace protocan
