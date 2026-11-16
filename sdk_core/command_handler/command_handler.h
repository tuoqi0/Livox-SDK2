
#ifndef COMMAND_HANDLER_H_
#define COMMAND_HANDLER_H_

#include <memory>
#include <map>
#include <mutex>

#include "base/command_callback.h"
#include "base/io_thread.h"

#include "comm/protocol.h"
#include "comm/define.h"

#include "livox_lidar_def.h"
#include "device_manager.h"

namespace livox {
namespace lidar {

class CommandHandler {
 public:
  CommandHandler(DeviceManager* device_manager) : device_manager_(device_manager) {}
  ~CommandHandler() {}

  virtual bool Init(bool is_view) = 0;
  virtual bool Init(const std::map<uint32_t, LivoxLidarCfg>& custom_lidars_cfg_map) = 0;

  virtual void Handle(const uint32_t handle, uint16_t lidar_port, const Command& command) = 0;
  virtual void UpdateLidarCfg(const ViewLidarIpInfo& view_lidar_info) = 0;
  virtual void UpdateLidarCfg(const uint32_t handle, const uint16_t lidar_cmd_port) = 0;
  virtual livox_status SendCommand(const Command& command) = 0;
  virtual livox_status SendLoggerCommand(const Command &command) = 0;
 protected:
  DeviceManager* device_manager_;
};

}  // namespace livox
} // namespace lidar

#endif  // COMMAND_HANDLER_H_
