#include "protocan/can_frame.hpp"

#include <algorithm>

namespace protocan
{

uint32_t encode_extended_id(const ExtendedId & eid)
{
  uint32_t id = 0;
  id |= (static_cast<uint32_t>(eid.function_code) & 0x0F) << 25;
  id |= (static_cast<uint32_t>(eid.src_dev) & 0x0F) << 21;
  id |= (static_cast<uint32_t>(eid.src_node) & 0x3F) << 15;
  id |= (static_cast<uint32_t>(eid.dst_dev) & 0x0F) << 11;
  id |= (static_cast<uint32_t>(eid.dst_node) & 0x3F) << 5;
  id |= (static_cast<uint32_t>(eid.context) & 0x1F);
  return id;
}

ExtendedId decode_extended_id(uint32_t raw_id)
{
  ExtendedId eid;
  eid.function_code = static_cast<FunctionCode>((raw_id >> 25) & 0x0F);
  eid.src_dev = static_cast<uint8_t>((raw_id >> 21) & 0x0F);
  eid.src_node = static_cast<uint8_t>((raw_id >> 15) & 0x3F);
  eid.dst_dev = static_cast<uint8_t>((raw_id >> 11) & 0x0F);
  eid.dst_node = static_cast<uint8_t>((raw_id >> 5) & 0x3F);
  eid.context = static_cast<uint8_t>(raw_id & 0x1F);
  return eid;
}

CanFrame make_extended_frame(const ExtendedId & eid, const uint8_t * payload, uint8_t len)
{
  CanFrame frame;
  frame.id = encode_extended_id(eid);
  frame.is_extended = true;
  frame.is_fd = true;
  frame.dlc = std::min(len, static_cast<uint8_t>(64));
  frame.clear_data();
  if (payload && len > 0) {
    std::memcpy(frame.data.data(), payload, frame.dlc);
  }
  return frame;
}

CanFrame make_standard_frame(uint16_t pdo_id, const uint8_t * payload, uint8_t len)
{
  CanFrame frame;
  frame.id = static_cast<uint32_t>(pdo_id & 0x7FF);
  frame.is_extended = false;
  frame.is_fd = true;
  frame.dlc = std::min(len, static_cast<uint8_t>(64));
  frame.clear_data();
  if (payload && len > 0) {
    std::memcpy(frame.data.data(), payload, frame.dlc);
  }
  return frame;
}

}  // namespace protocan
