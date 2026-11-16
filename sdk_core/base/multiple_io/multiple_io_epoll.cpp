
#include "multiple_io_epoll.h"
#ifdef HAVE_EPOLL
namespace livox {
namespace lidar {

int GetEvent (FdEvent event) {
  FdEvent rv = 0;
  if (event & READBLE_EVENT)
    rv |= EPOLLIN;
  if (event & WRITABLE_EVENT)
    rv |= EPOLLOUT;
  return rv;
}

bool MultipleIOEpoll::PollCreate(int size) {
  max_poll_size_ = size + 1;
  epoll_fd_ = epoll_create(max_poll_size_);
  if (epoll_fd_ < 0) {
    return false;
  }
  pollset_.reset(new struct epoll_event[size]);
  WakeUpInit();
  return true;
}

void MultipleIOEpoll::PollDestroy() {
  WakeUpUninit();
  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
    epoll_fd_ = -1;
  }
}

bool MultipleIOEpoll::PollSetAdd(PollFd poll_fd) {
  if (max_poll_size_ <= (int)descriptors_.size()) {
    return false;
  }
  struct epoll_event ee = {0};
  ee.events = GetEvent(poll_fd.event);
  ee.data.fd = poll_fd.fd;
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, poll_fd.fd, &ee) == -1) {
      return false;
  }

  int fd = poll_fd.fd;
  descriptors_[fd] = poll_fd;
  return true;
}

bool MultipleIOEpoll::PollSetRemove(PollFd poll_fd) {
  int fd = poll_fd.fd;
  struct epoll_event ee = {0};
  epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, &ee);
  if (descriptors_.find(fd) != descriptors_.end()) {
      descriptors_.erase(fd);
  }
  return true;
}

void MultipleIOEpoll::Poll(int time_out) {
  int ret = epoll_wait(epoll_fd_, pollset_.get(), (int)descriptors_.size(),
                    time_out);
  if (ret > 0) {
    for (int i =0; i< ret; i++) {
      FdEvent fd_event = NONE_EVENT;
      if (pollset_[i].events & EPOLLIN) {
        fd_event |= READBLE_EVENT;
      }
      if (pollset_[i].events & EPOLLOUT) {
        fd_event |= WRITABLE_EVENT;
      }
      int fd = pollset_[i].data.fd;
      if (descriptors_.find(fd) != descriptors_.end()) {
        PollFd pollfd =  descriptors_[fd];
        pollfd.event_callback(fd_event);
      }
    }
  }
  CheckTimer();
}

} // namespace lidar
}  // namespace livox

#endif  // HAVE_EPOLL
