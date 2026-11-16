
#include "multiple_io_select.h"
#include <thread>

#ifdef HAVE_SELECT
namespace livox {
namespace lidar {

bool MultipleIOSelect:: PollCreate(int size) {
  FD_ZERO(&rfds_);
  FD_ZERO(&wfds_);
  //wake up fd + 1
  max_poll_size_ = size + 1;
  WakeUpInit();
  return true;
}

void MultipleIOSelect::PollDestroy() {
  WakeUpUninit();
  FD_ZERO(&rfds_);
  FD_ZERO(&wfds_);
}

bool MultipleIOSelect:: PollSetAdd(PollFd poll_fd) {
  if (max_poll_size_ <= (int)descriptors_.size()) {
    return false;
  }
  int fd = poll_fd.fd;
  descriptors_[fd] = poll_fd;
  if (max_fd_ < fd) {
    max_fd_ = fd;
  }
  if (poll_fd.event & READBLE_EVENT) {
    FD_SET(fd, &rfds_);
  }
  if (poll_fd.event & WRITABLE_EVENT) {
    FD_SET(fd, &wfds_);
  }
  return true;
}

bool MultipleIOSelect:: PollSetRemove(PollFd poll_fd) {
  int fd = poll_fd.fd;
  if (descriptors_.find(fd) != descriptors_.end()) {
    descriptors_.erase(fd);
  }
  FD_CLR(fd, &rfds_);
  FD_CLR(fd, &wfds_);
  if (max_fd_ <= fd) {
    max_fd_--;
  }
  return true;
}

void MultipleIOSelect:: Poll(int time_out) {
  fd_set readset, writeset;
  struct timeval tv, *tvptr;
  if (descriptors_.size() == 0) {
    if (time_out > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(time_out));
      return;
    }
    return ;
  }

  if (time_out < 0) {
    tvptr = nullptr;
  } else {
    tv.tv_sec = (long)time_out / 1000;
    tv.tv_usec = (long)time_out % 1000;
    tvptr = &tv;
  }

  memcpy(&readset, &rfds_, sizeof(fd_set));
  memcpy(&writeset, &wfds_, sizeof(fd_set));

  int rv = select(max_fd_ + 1, &readset, &writeset, nullptr, tvptr);
  if (rv > 0) {
    for (auto& descriptor : descriptors_) {
      int fd = descriptor.first;
      FdEvent fd_event = NONE_EVENT;
      if (FD_ISSET(fd, &readset)) {
        fd_event |= READBLE_EVENT;
      }
      if (FD_ISSET(fd, &writeset)) {
        fd_event |= WRITABLE_EVENT;
      }
      if (fd_event != NONE_EVENT) {
        PollFd pollfd =  descriptor.second;
        pollfd.event_callback(fd_event);
      }
    }
  }
  CheckTimer();
}

} // namespace lidar
}  // namespace livox

#endif // HAVE_SELECT
