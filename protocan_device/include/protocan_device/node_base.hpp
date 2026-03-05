#pragma once

#include <cstddef>
#include <cstdint>

#include "protocan/can_frame.hpp"
#include "protocan/types.hpp"

namespace protocan::device
{

static constexpr uint8_t kMaxPdoPerNode = 8;

struct PdoRxEntry
{
  uint16_t pdo_id;
  uint8_t topic_index;
  uint8_t offset;
  uint8_t size;
};

struct PdoTxEntry
{
  uint16_t pdo_id;
  uint8_t topic_index;
  uint16_t period_ms;
  uint32_t last_tx_ms;
};

/// 全Node Type基底クラス。protoc-gen-protocanが生成したNodeクラスが継承。
class NodeBase
{
public:
  NodeBase(
    uint8_t local_id, const char * instance_name, const uint8_t * descriptor_blob,
    size_t blob_size, uint32_t schema_hash);

  uint8_t local_id() const { return local_id_; }
  const char * instance_name() const { return instance_name_; }
  uint32_t schema_hash() const { return schema_hash_; }
  const uint8_t * descriptor_blob() const { return descriptor_blob_; }
  size_t descriptor_blob_size() const { return blob_size_; }

  uint8_t pdo_tx_count() const { return pdo_tx_count_; }
  PdoTxEntry & pdo_tx_at(uint8_t i) { return pdo_tx_[i]; }

  uint8_t pdo_rx_count() const { return pdo_rx_count_; }
  const PdoRxEntry & pdo_rx_at(uint8_t i) const { return pdo_rx_[i]; }

  // Device から PDO_CFG 受信時に呼ばれる
  Status add_pdo_rx(const PdoRxEntry & e);
  Status add_pdo_tx(const PdoTxEntry & e);
  void clear_pdo(uint16_t pdo_id);   // COMMIT action=DELETE 時
  void reset_pdos();                  // RESET_NODE 時

  // ------- イベント駆動 PDO 送信用コールバック登録 -------

  using SendPdoFn = protocan::Status (*)(uint16_t pdo_id, const uint8_t * data, uint8_t len, void *);

  /// Device::add_node() が呼び出す。生成コードが override して send_pdo_fn_ を保存する。
  virtual void set_send_pdo(SendPdoFn /*fn*/, void * /*ctx*/) {}

  // ------- 生成コードがオーバーライドする純仮想メソッド -------

  /// PDO受信 (RX Topic)
  virtual void on_pdo_rx(uint16_t pdo_id, const uint8_t * data, uint8_t len) = 0;

  /// PDO送信バッファ充填 (TX Topic)
  virtual uint8_t fill_pdo_tx(uint16_t pdo_id, uint8_t * buf, uint8_t max_len) = 0;

  /// PARAM_READ_REQ
  virtual Status on_param_read(uint8_t idx, uint8_t * out, uint8_t & out_size) = 0;

  /// PARAM_WRITE_REQ
  virtual Status on_param_write(uint8_t idx, const uint8_t * data, uint8_t size) = 0;

  /// SERVICE_REQ
  virtual Status on_service_req(
    uint8_t svc_idx, const uint8_t * req, uint8_t req_size, uint8_t * res,
    uint8_t & res_size) = 0;

protected:
  uint8_t local_id_;
  const char * instance_name_;
  const uint8_t * descriptor_blob_;
  size_t blob_size_;
  uint32_t schema_hash_;

  PdoRxEntry pdo_rx_[kMaxPdoPerNode];
  uint8_t pdo_rx_count_ = 0;
  PdoTxEntry pdo_tx_[kMaxPdoPerNode];
  uint8_t pdo_tx_count_ = 0;
};

}  // namespace protocan::device
