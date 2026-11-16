
#ifndef LIVOX_SDK_PROTOCOL_H_
#define LIVOX_SDK_PROTOCOL_H_

#include <stdint.h>
#include "comm/protocol.h"
#include "FastCRC/FastCRC.h"

namespace livox {
namespace lidar {

typedef enum { kSdkVerNone, kSdkVer0, kSdkVer1 } SdkVersion;

#pragma pack(1)

typedef struct {
  uint8_t sof;
  uint8_t version;
  uint16_t length;
  uint32_t seq_num;
  uint16_t cmd_id;
  uint8_t cmd_type;
  uint8_t sender_type;
  char rsvd[6];
  uint16_t crc16_h;
  uint32_t crc32_d;
} SdkPreamble;

typedef struct {
  uint8_t sof;
  uint8_t version;
  uint16_t length;
  uint32_t seq_num;
  uint16_t cmd_id;
  uint8_t cmd_type;
  uint8_t sender_type;
  char rsvd[6];
  uint16_t crc16_h;
  uint32_t crc32_d;
  uint8_t data[1];
} SdkPacket;

#pragma pack()

class SdkProtocol : public Protocol {
 public:
  SdkProtocol();

  ~SdkProtocol();

  bool ParsePacket(uint8_t *i_buf, uint32_t buf_size, CommPacket *o_packet);

  int32_t Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len, const CommPacket &i_packet);

  uint32_t GetPreambleLen();

  uint32_t GetPacketWrapperLen();

  uint32_t GetPacketLen(uint8_t *buf);

  bool CheckPreamble(uint8_t *buf, uint32_t buf_size);
 private:
  FastCRC16 crc_16_;
  FastCRC32 crc_32_;
};
} // namespace lidar
}  // namespace livox
#endif  // LIVOX_SDK_PROTOCOL_H_
