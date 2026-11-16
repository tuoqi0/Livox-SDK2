
#ifndef MULTIPLE_IO_FACTORY_H_
#define MULTIPLE_IO_FACTORY_H_

#include "multiple_io_base.h"
#include "multiple_io_epoll.h"
#include "multiple_io_kqueue.h"
#include "multiple_io_select.h"
#include "multiple_io_poll.h"
#include <memory>

namespace livox {
namespace lidar {

class MultipleIOFactory {
 public:
  static std::unique_ptr<MultipleIOBase> CreateMultipleIO() {
#if defined(HAVE_EPOLL)
    return std::unique_ptr<MultipleIOBase>(new MultipleIOEpoll());
#elif defined(HAVE_KQUEUE)
    return std::unique_ptr<MultipleIOBase>(new MultipleIOKqueue());
#elif defined(HAVE_SELECT)
    return std::unique_ptr<MultipleIOBase>(new MultipleIOSelect());
#elif defined(HAVE_POLL)
    return std::unique_ptr<MultipleIOBase>(new MultipleIOPoll());
#else
    return nullptr;
#endif
  }
};

} // namespace lidar
}  // namespace livox

#endif  // MULTIPLE_IO_FACTORY_H_