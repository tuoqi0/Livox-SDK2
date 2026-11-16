
#ifndef DEBUG_POINT_CLOUD_HANDLER_H_
#define DEBUG_POINT_CLOUD_HANDLER_H_

#include "base/io_thread.h"
#include "base/noncopyable.h"
#include "base/logging.h"

#include "command_handler/command_impl.h"
#include "comm/define.h"

#include <string>
#include <cstdio>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>

namespace livox {
namespace lidar {

class DebugPointCloudHandler {
 public:
  DebugPointCloudHandler(std::uint32_t handle, std::string sn, std::uint8_t dev_type, std::string path);
  ~DebugPointCloudHandler();
  bool StoreData(uint8_t* buf, uint32_t buf_size);
  void WriteData();
  bool Enable(bool enable);

 private:
  std::uint32_t         handle_;
  std::string           sn_;
  std::uint8_t          dev_type_;
  std::uint64_t         file_size_{0};
  std::string           file_path_{""};
  std::string           file_name_;
  std::atomic<bool>     enable_{false};

  std::vector<uint8_t>  data_;
  std::mutex            data_mutex_;
  std::condition_variable cv_;

  std::shared_ptr<std::thread>  thread_ptr_{nullptr};
  FILE*                         file_handle_{nullptr};

  static constexpr uint64_t max_file_size_ = {4ULL * 1024 * 1024 * 1024};
};

} // namespace lidar
}  // namespace livox

#endif  // DEBUG_POINT_CLOUD_HANDLER_H_
