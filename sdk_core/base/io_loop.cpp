
#include "io_loop.h"
#include <functional>
#include <mutex>
#include <iostream>
#include <algorithm>
#include "logging.h"

using std::lock_guard;
using std::mutex;
using std::vector;
using std::chrono::steady_clock;

namespace livox {
namespace lidar {

bool IOLoop::Init() {
  auto multiple_io = MultipleIOFactory::CreateMultipleIO();
  if (!multiple_io) {
    LOG_ERROR("Creat Multiple IO Failed!");
    return false;
  }
  multiple_io_base_ = std::move(multiple_io);
  if (!multiple_io_base_->PollCreate(OPEN_MAX_POLL)) {
    LOG_ERROR("Poll Create Failed!");
    return false;
  }
  return true;
}

void IOLoop::Uninit() {
  if (multiple_io_base_) {
    multiple_io_base_->PollDestroy();
    multiple_io_base_.reset();
  }
}

void IOLoop::AddDelegate(socket_t sock, IOLoop::IOLoopDelegate *delegate, void *data) {
  PostTask(std::bind(&IOLoop::AddDelegateAsync, this, sock, delegate, data));
}

void IOLoop::RemoveDelegate(socket_t sock, IOLoopDelegate *) {
  PostTask(std::bind(&IOLoop::RemoveDelegateAsync, this, sock));
}

void IOLoop::Loop() {
  multiple_io_base_->Poll(POLL_TIMEOUT);

  vector<IOLoopTask> tasks;
  {
    lock_guard<mutex> lock(mutex_);
    tasks.swap(pending_tasks_);
  }

  for (auto &task : tasks) {
    task();
  }
}

 bool IOLoop::Wakeup() {
  if (multiple_io_base_) {
    multiple_io_base_->PollWakeUp();
  }
  return true;
 }

void IOLoop::PostTask(const IOLoopTask &task) {
  {
    lock_guard<mutex> lock(mutex_);
    pending_tasks_.push_back(task);
  }
  Wakeup();
}

void IOLoop::AddDelegateAsync(socket_t sock, IOLoop::IOLoopDelegate *delegate, void *data) {
  PollFd pollfd = {};
  pollfd.fd = sock;
  pollfd.event = READBLE_EVENT;
  pollfd.event_callback = [=](FdEvent event) {
    if (event & READBLE_EVENT) {
      if (delegate) {
        delegate->OnData(sock, data);
      }
    }
  };
  if (enable_timer_) {
    pollfd.timer_callback = [=](TimePoint t) {
      if (delegate) {
        delegate->OnTimer(t);
      }
    };
  }
  if (enable_wake_) {
    pollfd.wake_callback = [=]() {
      if (delegate) {
        delegate->OnWake();
      }
    };
  }
  multiple_io_base_->PollSetAdd(pollfd);
}

void IOLoop::RemoveDelegateAsync(socket_t sock) {
  PollFd pollfd = {};
  pollfd.fd = sock;
  pollfd.event = READBLE_EVENT;
  multiple_io_base_->PollSetRemove(pollfd);
}

} // namespace lidar
}  // namespace livox
