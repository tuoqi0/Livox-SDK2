
#include "io_thread.h"

namespace livox {
namespace lidar {

IOThread::~IOThread() {
  Join();
  Uninit();
}

void IOThread::ThreadFunc() {
  if (!loop_) {
    return;
  }
  while (!IsQuit()) {
    loop_->Loop();
  }
}

bool IOThread::Init(bool enable_timer, bool enable_wake) {
  loop_ = std::make_shared<IOLoop>(enable_timer, enable_wake);
  return loop_->Init();
}

void IOThread::Uninit() {
  if (loop_) {
    loop_->Uninit();
  }
}

} // namespace lidar
}  // namespace livox
