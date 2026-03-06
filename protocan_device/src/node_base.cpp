#include "protocan_device/node_base.hpp"

namespace protocan::device
{

NodeBase::NodeBase(
  uint8_t local_id, const char * instance_name, const uint8_t * descriptor_blob,
  size_t blob_size, uint32_t schema_hash)
: local_id_(local_id),
  instance_name_(instance_name),
  descriptor_blob_(descriptor_blob),
  blob_size_(blob_size),
  schema_hash_(schema_hash)
{
}

Status NodeBase::add_pdo_rx(const PdoRxEntry & e)
{
  if (pdo_rx_count_ >= kMaxPdoPerNode) {
    return Status::NO_RESOURCE;
  }
  pdo_rx_[pdo_rx_count_++] = e;
  return Status::OK;
}

Status NodeBase::add_pdo_tx(const PdoTxEntry & e)
{
  if (pdo_tx_count_ >= kMaxPdoPerNode) {
    return Status::NO_RESOURCE;
  }
  pdo_tx_[pdo_tx_count_++] = e;
  return Status::OK;
}

void NodeBase::clear_pdo(uint16_t pdo_id)
{
  // Remove all RX entries with matching pdo_id (compact the array)
  uint8_t dst = 0;
  for (uint8_t i = 0; i < pdo_rx_count_; ++i) {
    if (pdo_rx_[i].pdo_id != pdo_id) {
      pdo_rx_[dst++] = pdo_rx_[i];
    }
  }
  pdo_rx_count_ = dst;

  // Remove all TX entries with matching pdo_id
  dst = 0;
  for (uint8_t i = 0; i < pdo_tx_count_; ++i) {
    if (pdo_tx_[i].pdo_id != pdo_id) {
      pdo_tx_[dst++] = pdo_tx_[i];
    }
  }
  pdo_tx_count_ = dst;
}

void NodeBase::reset_pdos()
{
  pdo_rx_count_ = 0;
  pdo_tx_count_ = 0;
}

Status NodeBase::request_pdo_tx(uint16_t pdo_id)
{
  bool found = false;
  for (uint8_t i = 0; i < pdo_tx_count_; ++i) {
    if (pdo_tx_[i].pdo_id == pdo_id) {
      pdo_tx_[i].tx_requested = true;
      found = true;
    }
  }
  return found ? Status::OK : Status::NOT_FOUND;
}

bool NodeBase::is_pdo_tx_requested(uint16_t pdo_id) const
{
  for (uint8_t i = 0; i < pdo_tx_count_; ++i) {
    if (pdo_tx_[i].pdo_id == pdo_id && pdo_tx_[i].tx_requested) {
      return true;
    }
  }
  return false;
}

void NodeBase::clear_pdo_tx_request(uint16_t pdo_id)
{
  for (uint8_t i = 0; i < pdo_tx_count_; ++i) {
    if (pdo_tx_[i].pdo_id == pdo_id) {
      pdo_tx_[i].tx_requested = false;
    }
  }
}

}  // namespace protocan::device
