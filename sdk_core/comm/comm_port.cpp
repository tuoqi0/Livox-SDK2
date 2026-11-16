
#include "comm/comm_port.h"
#include <stdio.h>
#include <string.h>
#include <iostream>

#include "sdk_protocol.h"

namespace livox {
namespace lidar {

CommPort::CommPort() {
  protocol_ = new SdkProtocol();
}

CommPort::~CommPort() {
  if (protocol_) {
    delete protocol_;
  }
}

int32_t CommPort::Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len, const CommPacket &i_packet) {
  return protocol_->Pack(o_buf, o_buf_size, o_len, i_packet);
}

bool CommPort::ParseCommStream(uint8_t *buf, uint32_t buf_size, CommPacket *o_pack) {
  if (!(protocol_->CheckPreamble(buf, buf_size))) {
    printf("Comm Port Check Preamble error.\n");
    return false;
  }
  return protocol_->ParsePacket(buf, buf_size, o_pack);
}

} // namespace lidar
}  // namespace livox
