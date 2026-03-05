#include "socketcan_interface.hpp"

#include <cerrno>
#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

SocketCanInterface::SocketCanInterface(std::string iface_name)
: iface_name_(std::move(iface_name))
{
}

SocketCanInterface::~SocketCanInterface()
{
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
  }
}

protocan::Status SocketCanInterface::open()
{
  sock_fd_ = ::socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_fd_ < 0) {
    return protocan::Status::ERROR;
  }

  // Enable CAN FD frames
  int enable = 1;
  if (::setsockopt(sock_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
    return protocan::Status::ERROR;
  }

  // Resolve interface index
  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, iface_name_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
    return protocan::Status::ERROR;
  }

  // Bind to interface
  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(sock_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
    return protocan::Status::ERROR;
  }

  return protocan::Status::OK;
}

protocan::Status SocketCanInterface::close()
{
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
  return protocan::Status::OK;
}

bool SocketCanInterface::is_open() const
{
  return sock_fd_ >= 0;
}

protocan::Status SocketCanInterface::send(const protocan::CanFrame & frame)
{
  struct canfd_frame cf {};

  if (frame.is_extended) {
    cf.can_id = (frame.id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  } else {
    cf.can_id = frame.id & CAN_SFF_MASK;
  }
  cf.flags = CANFD_BRS;
  cf.len = frame.dlc;
  std::copy(frame.data.begin(), frame.data.begin() + frame.dlc, cf.data);

  ssize_t n = ::write(sock_fd_, &cf, CANFD_MTU);
  return (n == CANFD_MTU) ? protocan::Status::OK : protocan::Status::ERROR;
}

std::optional<protocan::CanFrame> SocketCanInterface::receive()
{
  struct canfd_frame cf {};
  ssize_t n = ::recv(sock_fd_, &cf, sizeof(cf), MSG_DONTWAIT);
  if (n < 0) {
    // No frame available
    return std::nullopt;
  }

  protocan::CanFrame frame;
  frame.is_extended = (cf.can_id & CAN_EFF_FLAG) != 0;
  frame.id = cf.can_id & (frame.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  frame.is_fd = (n == CANFD_MTU);
  frame.dlc = cf.len;
  std::copy(cf.data, cf.data + cf.len, frame.data.begin());
  return frame;
}
