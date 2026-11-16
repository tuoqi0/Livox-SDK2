
#ifndef COMM_PROTOCOL_H_
#define COMM_PROTOCOL_H_

#include <stdint.h>

namespace livox {
namespace lidar {

typedef struct CommPacket CommPacket;

typedef int (*RequestPackCb)(CommPacket *packet);

typedef enum { kRequestPack, kAckPack, kMsgPack } PacketType;

typedef enum { kLidarSdk, kRsvd1, kProtocolUndef } ProtocolType;

typedef enum { kNoNeed, kNeedAck, kDelayAck } NeedAckType;

typedef enum { kParseSuccess, kParseFail } ParseResult;

typedef struct LogCommPacket {
  uint8_t packet_type;
  uint8_t protocol;
  uint8_t protocol_version;
  uint8_t cmd_set;
  uint32_t cmd_code;
  uint32_t sender;
  uint32_t sub_sender;
  uint32_t receiver;
  uint32_t sub_receiver;
  uint32_t seq_num;
  uint8_t *data;
  uint16_t data_len;
  uint32_t padding;
} LogCommPacket;

typedef struct CommPacket {
  uint8_t protocol;
  uint8_t version;
  uint32_t seq_num;
  uint16_t cmd_id;
  uint8_t cmd_type;
  uint8_t sender_type;
  uint8_t* data;
  uint16_t data_len;
} CommPacket;

class Protocol {
 public:
  virtual ~Protocol(){};

  virtual bool ParsePacket(uint8_t *i_buf, uint32_t i_len, CommPacket *o_packet) = 0;

  virtual int32_t Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len, const CommPacket &i_packet) = 0;

  virtual uint32_t GetPreambleLen() = 0;

  virtual uint32_t GetPacketWrapperLen() = 0;

  virtual uint32_t GetPacketLen(uint8_t *buf) = 0;

  virtual bool CheckPreamble(uint8_t *buf, uint32_t buf_size) = 0;

};

} // namespace lidar
}  // namespace livox
#endif  // COMM_PROTOCOL_H_
