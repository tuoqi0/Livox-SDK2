
#ifndef MULTIPLE_IO_POLL_H_
#define MULTIPLE_IO_POLL_H_

#include "multiple_io_base.h"
#include "livox_lidar_cfg.h"
#include <memory>


#ifdef HAVE_POLL

namespace livox {
namespace lidar {

class MultipleIOPoll : public MultipleIOBase {
 public:
  bool PollCreate(int size);
  bool PollSetAdd(PollFd poll_fd);
  bool PollSetRemove(PollFd poll_fd);
  void Poll(int timeout);
  void PollDestroy();
 private:
  std::unique_ptr<struct pollfd[]> pollset_;
  int pollset_num_ = 0;
  int max_poll_size_ = 0;
};

} // namespace lidar
}  // namespace livox

#endif  // HAVE_POLL
#endif  // MULTIPLE_IO_POLL_H_