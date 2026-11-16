
#include "multiple_io_kqueue.h"
#ifdef HAVE_KQUEUE

namespace livox {
namespace lidar {

bool MultipleIOKqueue::PollCreate(int size) {
  kqueue_fd_ = kqueue();
  if (kqueue_fd_ == -1) {
    return false;
  }

  int flags = 0;
  if ((flags = fcntl(kqueue_fd_, F_GETFD)) == -1) {
    close(kqueue_fd_);
    return false;
  }
  flags |= FD_CLOEXEC;
  if (fcntl(kqueue_fd_, F_SETFD, flags) == -1) {
    close(kqueue_fd_);
    return false;
  }
  max_poll_size_ = size + 1;
  set_size_ = 2 * max_poll_size_;
  kevent_set_.reset(new struct kevent[set_size_]);
  WakeUpInit();
  return true;
}

void MultipleIOKqueue::PollDestroy() {
  WakeUpUninit();
  if (kqueue_fd_ >= 0) {
    close(kqueue_fd_);
    kqueue_fd_ = -1;
  }
}

bool MultipleIOKqueue::PollSetAdd(PollFd poll_fd) {
  if (max_poll_size_ <= (int)descriptors_.size()) {
    return false;
  }
  int fd = poll_fd.fd;
  descriptors_[fd] = poll_fd;
  if (poll_fd.event & READBLE_EVENT) {
    EV_SET(&kevent_, fd, EVFILT_READ, EV_ADD, 0, 0, &descriptors_[fd].fd);
    if (kevent(kqueue_fd_, &kevent_, 1, nullptr, 0, nullptr) == -1) {
      descriptors_.erase(fd);
      return false;
    }
  }
  if (poll_fd.event & WRITABLE_EVENT) {
    EV_SET(&kevent_, fd, EVFILT_WRITE, EV_ADD, 0, 0, &descriptors_[fd].fd);
    if (kevent(kqueue_fd_, &kevent_, 1, nullptr, 0, nullptr) == -1) {
      descriptors_.erase(fd);
      return false;
    }
  }
  return true;
}

bool MultipleIOKqueue::PollSetRemove(PollFd poll_fd) {
  int fd = poll_fd.fd;
  bool result = true;
  if (descriptors_.find(fd) != descriptors_.end()) {
    do {
      if (descriptors_[fd].event & READBLE_EVENT) {
        EV_SET(&kevent_, fd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
        if (kevent(kqueue_fd_, &kevent_, 1, nullptr, 0, nullptr) == -1) {
          result = false;
          break;
        }
      }
      if (descriptors_[fd].event & WRITABLE_EVENT) {
        EV_SET(&kevent_, fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);
        if (kevent(kqueue_fd_, &kevent_, 1, nullptr, 0, nullptr) == -1) {
          result = false;
          break;
        }
      }
    } while(0);
    descriptors_.erase(fd);
  }
  return result;
}

void MultipleIOKqueue::Poll(int time_out) {
  struct timespec tv, *tvptr;

  if (time_out < 0) {
    tvptr = NULL;
  } else {
    tv.tv_sec = (long) time_out / 1000;
    tv.tv_nsec = (long) (time_out % 1000) * 1000000;
    tvptr = &tv;
  }

  int rv = kevent(kqueue_fd_, NULL, 0, kevent_set_.get(), set_size_, tvptr);
  if (rv > 0) {
    for (int i = 0; i < rv; i++) {
      int fd = *(int *)(kevent_set_[i].udata);
      if (descriptors_.find(fd) != descriptors_.end()) {
        PollFd pollfd =  descriptors_[fd];
        if (kevent_set_[i].filter == EVFILT_READ) {
          pollfd.event_callback(READBLE_EVENT);
        }
        if (kevent_set_[i].filter == EVFILT_WRITE) {
          pollfd.event_callback(WRITABLE_EVENT);
        }
      }
    }
  }
  CheckTimer();
}

} // namespace lidar
}  // namespace livox

#endif // HAVE_KQUEUE
