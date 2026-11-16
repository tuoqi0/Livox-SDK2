
#ifndef MULTIPLE_IO_BASE_H_
#define MULTIPLE_IO_BASE_H_

#include <map>
#include <functional>
#include <chrono>
#include <memory>
#include "base/wake_up/wake_up_pipe.h"

namespace livox {
namespace lidar {

#define NONE_EVENT 0       /* No events registered. */
#define READBLE_EVENT 1    /* when descriptor is readable. */
#define WRITABLE_EVENT 2   /* when descriptor is writeable. */

typedef std::chrono::steady_clock::time_point TimePoint;
typedef int FdEvent;

typedef struct {
  int fd;                                         /* File descriptor. */
  FdEvent event;                                  /* Read | Write Event to listen. */
  std::function<void(FdEvent)> event_callback;    /* Read or Write Event Callback. */
  std::function<void(TimePoint)> timer_callback;  /* Timer Event Callback. */
  std::function<void()> wake_callback;            /* WakeUp Event Callback. */
} PollFd;

class MultipleIOBase {
 public:
  MultipleIOBase() = default;
  virtual ~MultipleIOBase() = default;
  virtual bool PollCreate(int size) = 0;
  virtual void PollDestroy() = 0;
  virtual bool PollSetAdd(PollFd poll_fd) = 0;
  virtual bool PollSetRemove(PollFd poll_fd) = 0;
  virtual void Poll(int timeout) = 0;
  virtual void PollWakeUp();
 protected:
  virtual void CheckTimer();
  std::map<int, PollFd> descriptors_;
  TimePoint last_timeout_ = TimePoint();

  virtual void WakeUpInit();
  virtual void WakeUpUninit();
  std::unique_ptr<WakeUpPipe> wake_up_pipe_;
};

} // namespace lidar
}  // namespace livox

#endif // MULTIPLE_IO_BASE_H_
