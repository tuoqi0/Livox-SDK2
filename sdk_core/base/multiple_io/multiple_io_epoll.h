
#ifndef MULTIPLE_IO_EPOLL_H_
#define MULTIPLE_IO_EPOLL_H_

#include "multiple_io_base.h"
#include "livox_lidar_cfg.h"
#include <memory>

#ifdef HAVE_EPOLL

namespace livox {
namespace lidar {

class MultipleIOEpoll : public MultipleIOBase {
 public:
  bool PollCreate(int size);
  bool PollSetAdd(PollFd poll_fd);
  bool PollSetRemove(PollFd poll_fd);
  void Poll(int timeout);
  void PollDestroy();
 private:
  int epoll_fd_ = -1;
  std::unique_ptr<struct epoll_event[]> pollset_;
  int max_poll_size_ = 0;
};

} // namespace lidar
}  // namespace livox

#endif  // HAVE_EPOLL
#endif  // MULTIPLE_IO_EPOLL_H_
