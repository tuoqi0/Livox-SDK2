
#include "multiple_io_poll.h"

#ifdef HAVE_POLL

namespace livox {
namespace lidar {

int GetEvent (FdEvent event) {
  int rv = 0;
  if (event & READBLE_EVENT)
      rv |= POLLIN;
  if (event & WRITABLE_EVENT)
      rv |= POLLOUT;
  return rv;
}

bool MultipleIOPoll:: PollCreate(int size) {
  max_poll_size_ = size + 1;
  pollset_.reset(new struct pollfd[max_poll_size_]);
  WakeUpInit();
  return true;
}

bool MultipleIOPoll:: PollSetAdd(PollFd poll_fd) {
  if (max_poll_size_ <= (int)descriptors_.size()) {
    return false;
  }

  int fd = poll_fd.fd;
  descriptors_[fd] = poll_fd;

  struct pollfd fds;
  fds.fd = fd;
  fds.events = GetEvent(poll_fd.event);
  pollset_[pollset_num_] = fds;
  pollset_num_++;
  return true;
}

void MultipleIOPoll::PollDestroy() {
  WakeUpUninit();
  return;
}

bool MultipleIOPoll:: PollSetRemove(PollFd poll_fd) {
  int fd = poll_fd.fd;
  if (descriptors_.find(fd) != descriptors_.end()) {
    descriptors_.erase(fd);
  }

  for (int i = 0; i< pollset_num_; i++) {
    if (pollset_[i].fd == fd) {
      int dst = i;
      for (i++; i < pollset_num_ - 1; i++) {
        if (pollset_[i].fd == fd) {
          pollset_num_--;
        } else {
          pollset_[dst] = pollset_[i];
          dst++;
        }
      }
    }
  }
  return true;
}

void MultipleIOPoll:: Poll(int time_out) {
  int rv = poll(pollset_.get(), pollset_num_, time_out);
  if (rv > 0) {
    for (int i = 0; i < pollset_num_; i++) {
      FdEvent fd_event = NONE_EVENT;
      if (pollset_[i].revents & POLLIN) {
        fd_event |= READBLE_EVENT;
      }
      if (pollset_[i].revents & POLLOUT) {
        fd_event |= WRITABLE_EVENT;
      }
      int fd = pollset_[i].fd;
      if (descriptors_.find(fd) != descriptors_.end()) {
          PollFd pollfd =  descriptors_[fd];
          pollfd.event_callback(fd_event);
      }
      pollset_[i].revents = NONE_EVENT;
    }
  }
  CheckTimer();
}

} // namespace lidar
}  // namespace livox

#endif // HAVE_POLL
