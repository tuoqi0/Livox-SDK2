
#ifndef LIVOX_THREAD_BASE_H_
#define LIVOX_THREAD_BASE_H_
#include <atomic>
#include <thread>
#include <memory>
#include "noncopyable.h"

namespace livox {
namespace lidar {

class ThreadBase : public noncopyable {
 public:
  ThreadBase();
  virtual ~ThreadBase();
  virtual void ThreadFunc() = 0;
  bool Start();
  bool IsQuit() { return quit_; }

 protected:
  void Join();
  
 private:
  std::atomic_bool quit_;
  std::shared_ptr<std::thread> thread_;
};

} //namespace lidar
}  // namespace livox

#endif  // LIVOX_THREAD_BASE_H_
