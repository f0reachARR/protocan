#include "socketcan_device_interface.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <utility>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

SocketCanDeviceInterface::SocketCanDeviceInterface(std::string iface_name)
: iface_name_(std::move(iface_name))
{
}

SocketCanDeviceInterface::~SocketCanDeviceInterface()
{
  (void)close();
}

protocan::Status SocketCanDeviceInterface::open()
{
  if (sock_fd_ >= 0) {
    return protocan::Status::OK;
  }

  sock_fd_ = ::socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_fd_ < 0) {
    return protocan::Status::ERROR;
  }

  int enable_fd = 1;
  if (::setsockopt(sock_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
    (void)close();
    return protocan::Status::ERROR;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, iface_name_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0) {
    (void)close();
    return protocan::Status::ERROR;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(sock_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    (void)close();
    return protocan::Status::ERROR;
  }

  return protocan::Status::OK;
}

protocan::Status SocketCanDeviceInterface::close()
{
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
  return protocan::Status::OK;
}

bool SocketCanDeviceInterface::is_open() const
{
  return sock_fd_ >= 0;
}

protocan::Status SocketCanDeviceInterface::send(const protocan::CanFrame & frame)
{
  if (sock_fd_ < 0) {
    return protocan::Status::ERROR;
  }

  if (frame.is_fd) {
    if (frame.dlc > 64) {
      return protocan::Status::INVALID_ARGUMENT;
    }

    struct canfd_frame cf {};
    cf.can_id = frame.is_extended ? ((frame.id & CAN_EFF_MASK) | CAN_EFF_FLAG) : (frame.id & CAN_SFF_MASK);
    cf.len = frame.dlc;
    std::copy_n(frame.data.begin(), frame.dlc, cf.data);

    const ssize_t n = ::write(sock_fd_, &cf, CANFD_MTU);
    return (n == CANFD_MTU) ? protocan::Status::OK : protocan::Status::ERROR;
  }

  if (frame.dlc > 8) {
    return protocan::Status::INVALID_ARGUMENT;
  }

  struct can_frame cf {};
  cf.can_id = frame.is_extended ? ((frame.id & CAN_EFF_MASK) | CAN_EFF_FLAG) : (frame.id & CAN_SFF_MASK);
  cf.can_dlc = frame.dlc;
  std::copy_n(frame.data.begin(), frame.dlc, cf.data);

  const ssize_t n = ::write(sock_fd_, &cf, CAN_MTU);
  return (n == CAN_MTU) ? protocan::Status::OK : protocan::Status::ERROR;
}

bool SocketCanDeviceInterface::try_receive(protocan::CanFrame & out)
{
  if (sock_fd_ < 0) {
    return false;
  }

  struct canfd_frame cf {};
  const ssize_t n = ::recv(sock_fd_, &cf, CANFD_MTU, MSG_DONTWAIT);
  if (n < 0) {
    return errno == EAGAIN || errno == EWOULDBLOCK ? false : false;
  }
  if (n != CAN_MTU && n != CANFD_MTU) {
    return false;
  }

  out = {};
  out.is_extended = (cf.can_id & CAN_EFF_FLAG) != 0;
  out.id = cf.can_id & (out.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  out.is_fd = (n == CANFD_MTU);
  out.dlc = static_cast<uint8_t>(cf.len);
  const uint8_t copy_len = std::min<uint8_t>(out.dlc, static_cast<uint8_t>(out.data.size()));
  std::copy_n(cf.data, copy_len, out.data.begin());
  return true;
}
