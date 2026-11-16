
#ifndef LIVOX_IO_THREAD_H_
#define LIVOX_IO_THREAD_H_
#include <memory>
#include "io_loop.h"
#include "thread_base.h"

namespace livox {
namespace lidar {

class IOThread : public ThreadBase {
 public:
  IOThread() : loop_(nullptr) {}
  virtual ~IOThread();
  bool Init(bool enable_timer = true, bool enable_wake = true);
  std::weak_ptr<IOLoop> GetLoop() { return loop_; }
  void ThreadFunc();
  using ThreadBase::Join;

 private:
  void Uninit();
  std::shared_ptr<IOLoop> loop_;
};

} // namespace lidar
}  // namespace livox
#endif  // LIVOX_IO_THREAD_H_
