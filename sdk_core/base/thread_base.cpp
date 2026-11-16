
#include "thread_base.h"
#include <thread>
#include <memory>

namespace livox {
namespace lidar {


ThreadBase::ThreadBase() : quit_(false) {}

bool ThreadBase::Start() {
  quit_ = false;
  thread_ = std::make_shared<std::thread>(&ThreadBase::ThreadFunc, this);
  return true;
}

ThreadBase::~ThreadBase() {
  if (thread_) {
    Join();
  }
}

void ThreadBase::Join() {
  quit_ = true;
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
  thread_.reset();
}

} // namespace lidar
}  // namespace livox
