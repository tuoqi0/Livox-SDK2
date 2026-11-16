
#ifndef LIVOX_LOGGER_HANDLER_
#define LIVOX_LOGGER_HANDLER_

#include <string>
#include <algorithm>
#include <map>
#include <queue>
#include <mutex>
#include <thread>
#include <stdio.h>

#include "base/io_thread.h"
#include "base/noncopyable.h"

#include "command_handler/command_impl.h"
#include "comm/comm_port.h"
#include "comm/define.h"

namespace livox {
namespace lidar {

class LoggerHandler {
 public:
  struct CurrentFileInfo {
    uint8_t flag {0};
    uint8_t file_index {0};
    uint32_t trans_index {0};
    std::FILE* fp {nullptr};
    std::string file_name {""};
  };

  struct WriteBuffer {
    uint8_t log_type;
    uint8_t flag {0};
    uint8_t file_index {0};
    uint16_t data_length {0};        
    uint32_t trans_index {0};
    std::shared_ptr<uint8_t> data_ptr  = std::make_shared<uint8_t>(0);
  };

public:
  explicit LoggerHandler(std::string log_root_path, std::string serial_num) : 
    log_root_path_(log_root_path),
    serial_num_(serial_num),
    is_stop_write_(false),
    thread_ptr_(nullptr) {
  };

  ~LoggerHandler() {
    Destory();
  }

  void Init();
  void Destory();

  void StoreLogBag(DeviceLoggerFilePushRequest* req, uint8_t flag);
  void CreateFile(const WriteBuffer& write_buff);
  void WriteFile(const WriteBuffer& write_buff);
  void StopFile(const WriteBuffer& write_buff);

  void Write();
  void SaveToFile();
private:
  std::string log_root_path_;
  std::map <uint8_t, std::string> log_branch_path_;
  std::string serial_num_;
  std::map <uint8_t, CurrentFileInfo> current_files_;
  
  std::mutex queue_mutex_;
  std::queue<WriteBuffer> queue_;
  std::atomic<bool> is_stop_write_;
  std::shared_ptr<std::thread> thread_ptr_;
};

} // namespace lidar
}  // namespace livox

#endif  // LIVOX_LOGGER_HANDLER_
