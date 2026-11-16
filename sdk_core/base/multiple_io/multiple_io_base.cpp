
#include "multiple_io_base.h"

namespace livox {
namespace lidar {

void MultipleIOBase::CheckTimer() {
  TimePoint t = std::chrono::steady_clock::now();
  if (t - last_timeout_ > std::chrono::milliseconds(50)) {
    last_timeout_ = t;
    for (auto & descriptor : descriptors_) {
      PollFd pollfd =  descriptor.second;
      if (pollfd.timer_callback) {
        pollfd.timer_callback(t);
      }
    }
  }
}

void MultipleIOBase::WakeUpInit() {
  //Initialize wake up pipe
  wake_up_pipe_.reset(new WakeUpPipe());
  wake_up_pipe_->PipeCreate();
  //register wake_fd to multiple io
  PollFd wake_fd = {};
  wake_fd.fd = wake_up_pipe_->GetPipeOut();
  wake_fd.event = READBLE_EVENT;
  wake_fd.event_callback = [this](FdEvent event) {
    if (event & READBLE_EVENT) {
      if (wake_up_pipe_) {
        wake_up_pipe_->Drain();
      }
      for (auto & descriptor : descriptors_) {
        PollFd pollfd =  descriptor.second;
        if (pollfd.wake_callback) {
          pollfd.wake_callback();
        }
      }
    }
  };
  PollSetAdd(wake_fd);
}

void MultipleIOBase::WakeUpUninit() {
  if (wake_up_pipe_) {
    PollFd wake_fd = {};
    wake_fd.fd = wake_up_pipe_->GetPipeOut();
    wake_fd.event = READBLE_EVENT | WRITABLE_EVENT;
    PollSetRemove(wake_fd);
    wake_up_pipe_->PipeDestroy();
    wake_up_pipe_ = nullptr;
  }
}

void MultipleIOBase::PollWakeUp() {
  if (wake_up_pipe_) {
    wake_up_pipe_->WakeUp();
  }
  return;
}

} // namespace lidar
}  // namespace livox
