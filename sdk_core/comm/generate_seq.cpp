
#include "generate_seq.h"
#include <cstdint>

namespace livox {
namespace lidar {

uint32_t GenerateSeq::GetSeq() {
  static std::atomic<std::uint32_t> seq(1);
  uint32_t value = seq.load();
  uint32_t desired = 0;
  do {
    if (value == UINT16_MAX) {
      desired = 1;
    } else {
      desired = value + 1;
    }
  } while (!seq.compare_exchange_weak(value, desired));
  return desired;
}


} // namespace lidar
}  // namespace livox
