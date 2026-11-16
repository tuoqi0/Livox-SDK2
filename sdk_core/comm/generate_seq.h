
#ifndef LIVOX_GENERATE_SEQ_H_
#define LIVOX_GENERATE_SEQ_H_

#include <cstdint>
#include <memory>
#include <atomic>


namespace livox {
namespace lidar {


class GenerateSeq {
 public:
  static uint32_t GetSeq();

};

} // namespace lidar
}  // namespace livox
#endif  // GENERATE_SEQ_H_
