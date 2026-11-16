
#ifndef DEBUG_POINT_CLOUD_MANAGER_H_
#define DEBUG_POINT_CLOUD_MANAGER_H_

#include <functional>
#include <memory>
#include <mutex>
#include <condition_variable>

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include "debug_point_cloud_handler.h"

#include "base/io_thread.h"
#include "base/logging.h"
#include "comm/define.h"
#include "base/network/network_util.h"

namespace livox {
namespace lidar {

class DebugPointCloudManager {
 public:
  DebugPointCloudManager(const DebugPointCloudManager& other) = delete;
  DebugPointCloudManager& operator=(const DebugPointCloudManager& other) = delete;
  ~DebugPointCloudManager();

  void AddDevice(const uint32_t handle, const DetectionData* detection_data);
  void Handler(uint32_t handle, uint16_t lidar_port, uint8_t *buf, uint32_t buf_size);
  bool Enable(bool enable);
  void Destory();
  bool SetStorePath(std::string path);
  
  static DebugPointCloudManager& GetInstance();

 private:
  DebugPointCloudManager();

 private:
  std::atomic<bool> enable_{false};
  std::string path_;
  std::map<uint32_t, LidarDeviceInfo> devices_info_;
  std::map<uint32_t, std::shared_ptr<DebugPointCloudHandler>> handlers_;
};

} // namespace lidar
}  // namespace livox

#endif  // DEBUG_POINT_CLOUD_MANAGER_H_
