#pragma once

#include <optional>
#include <string>

#include "protocan/can_frame.hpp"
#include "protocan/can_interface.hpp"

class SocketCanInterface : public protocan::ICanInterface
{
public:
  explicit SocketCanInterface(std::string iface_name);
  ~SocketCanInterface() override;

  protocan::Status send(const protocan::CanFrame & frame) override;
  std::optional<protocan::CanFrame> receive() override;

  protocan::Status open() override;
  protocan::Status close() override;
  bool is_open() const override;

private:
  std::string iface_name_;
  int sock_fd_ = -1;
};
