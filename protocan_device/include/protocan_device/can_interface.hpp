#pragma once

#include "protocan/can_frame.hpp"
#include "protocan/types.hpp"

namespace protocan::device
{

/// MCU向けCAN HAL抽象インターフェース (std::optional不使用)
class ICanInterface
{
public:
  virtual ~ICanInterface() = default;

  virtual Status send(const CanFrame & frame) = 0;

  /// ノンブロッキング受信。フレームあり→true + out書込、なし→false
  virtual bool try_receive(CanFrame & out) = 0;
};

}  // namespace protocan::device
