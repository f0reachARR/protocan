#pragma once

#include <string>

#include "protocan_device/can_interface.hpp"

class SocketCanDeviceInterface : public protocan::device::ICanInterface
{
public:
  explicit SocketCanDeviceInterface(std::string iface_name);
  ~SocketCanDeviceInterface() override;

  protocan::Status open();
  protocan::Status close();
  bool is_open() const;

  protocan::Status send(const protocan::CanFrame & frame) override;
  bool try_receive(protocan::CanFrame & out) override;

private:
  std::string iface_name_;
  int sock_fd_ = -1;
};
