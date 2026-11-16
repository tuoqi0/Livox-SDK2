
#ifndef MULTIPLE_IO_KQUEUE_H_
#define MULTIPLE_IO_KQUEUE_H_

#include "multiple_io_base.h"
#include "livox_lidar_cfg.h"
#ifdef HAVE_KQUEUE

namespace livox {
namespace lidar {

class MultipleIOKqueue : public MultipleIOBase {
 public:
  bool PollCreate(int size);
  bool PollSetAdd(PollFd poll_fd);
  bool PollSetRemove(PollFd poll_fd);
  void Poll(int timeout);
  void PollDestroy();
 private:
  int kqueue_fd_ = -1;
  struct kevent kevent_ = {};
  std::unique_ptr<struct kevent[]> kevent_set_;
  int max_poll_size_ = 0;
  int set_size_ = 0;
};

} // namespace lidar
}  // namespace livox

#endif  // HAVE_KQUEUE
#endif  // MULTIPLE_IO_KQUEUE_H_