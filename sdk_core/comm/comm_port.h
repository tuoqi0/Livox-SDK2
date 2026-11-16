
#ifndef COMM_COMM_PORT_H_
#define COMM_COMM_PORT_H_

#include <stdint.h>
#include "protocol.h"

namespace livox {
namespace lidar {
const uint32_t kCacheSize = 8192;

class CommPort {
 public:
  CommPort();

  ~CommPort();

  int32_t Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len, const CommPacket &i_packet);

  bool ParseCommStream(uint8_t *o_buf, uint32_t buf_size, CommPacket *o_pack);
 private:
  Protocol *protocol_;
};

} // namespace lidar
}  // namespace livox
#endif  // COMM_COMM_PORT_H_
