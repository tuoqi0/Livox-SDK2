
#ifndef WIN32
#include "base/wake_up/wake_up_pipe.h"
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

namespace livox {
namespace lidar {

WakeUpPipe::~WakeUpPipe() {
  PipeDestroy();
}

bool WakeUpPipe::WakeUp() {
  char ch = '1';
  ssize_t nbytes = sizeof(ch);
  if (pipe_in_ >= 0) {
    if (nbytes != write(pipe_in_, &ch, nbytes)) {
      return false;
    }
  }
  return true;
}

bool WakeUpPipe::Drain() {
  char ch[512];
  size_t size = sizeof(ch);
  if (pipe_out_ >= 0) {
    ssize_t ret = read(pipe_out_, ch, size);
    if (ret < 0) {
      return false;
    }
  }
  return true;
}

bool WakeUpPipe::PipeDestroy() {
  if (pipe_in_ >= 0) {
    close(pipe_in_);
    pipe_in_ = -1;
  }
  if (pipe_out_ >= 0) {
    close(pipe_out_);
    pipe_out_ = -1;
  }
  return true;
}

bool WakeUpPipe::PipeCreate() {
  bool status = false;
  //in filedes[0]
  //out filedes[1]
  int filedes[2]= {};
  if (pipe(filedes) == -1) {
    return false;
  }
  do {
    int flags = 0;
    if ((flags = fcntl(filedes[0], F_GETFD)) == -1) {
      break;
    }

    flags |= FD_CLOEXEC;
    if (fcntl(filedes[0], F_SETFD, flags) == -1) {
      break;
    }

    flags = 0;
    if ((flags = fcntl(filedes[1], F_GETFD)) == -1) {
      break;
    }

    flags |= FD_CLOEXEC;
    if (fcntl(filedes[1], F_SETFD, flags) == -1) {
      break;
    }
    status = true;
  } while(0);

  if (!status) {
    if (filedes[0] >= 0) {
      close(filedes[0]);
    }
    if (filedes[1] >= 0) {
      close(filedes[1]);
    }
    return false;
  }
  pipe_out_ = filedes[0];
  pipe_in_ = filedes[1];
  return true;
}

} // namespace lidar
}  // namespace livox

#endif  // WIN32
