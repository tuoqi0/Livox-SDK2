
#ifndef MULTIPLE_IO_SELECT_H_
#define MULTIPLE_IO_SELECT_H_

#include "multiple_io_base.h"
#include "livox_lidar_cfg.h"

#ifdef HAVE_SELECT

namespace livox {
namespace lidar {

class MultipleIOSelect : public MultipleIOBase {
 public:
  bool PollCreate(int size);
  bool PollSetAdd(PollFd poll_fd);
  bool PollSetRemove(PollFd poll_fd);
  void Poll(int timeout);
  void PollDestroy();
 private:
  int max_fd_ = -1;
  fd_set rfds_;
  fd_set wfds_;
  int max_poll_size_ = 0;
};

} // namespace lidar
} // namespace livox

#endif // HAVE_SELECT
#endif // MULTIPLE_IO_SELECT_H_