
#ifndef AVIA2_COMMAND_HANDLER_H_
#define AVIA2_COMMAND_HANDLER_H_

#include <memory>
#include <map>
#include <list>
#include <mutex>

#include "base/command_callback.h"
#include "base/io_thread.h"

#include "comm/protocol.h"
#include "comm/comm_port.h"
#include "comm/define.h"

#include "livox_lidar_def.h"
#include "device_manager.h"

#include "command_handler.h"

namespace livox {
namespace lidar {  

class Avia2CommandHandler : public CommandHandler {
 public:
  Avia2CommandHandler(DeviceManager* device_manager);
  ~Avia2CommandHandler() {}
  virtual bool Init(bool is_view);

  virtual bool Init(const std::map<uint32_t, LivoxLidarCfg>& custom_lidars_cfg_map);
  virtual void Handle(const uint32_t handle, uint16_t lidar_port, const Command& command);
  virtual void UpdateLidarCfg(const ViewLidarIpInfo& view_lidar_info);
  virtual void UpdateLidarCfg(const uint32_t handle, const uint16_t lidar_cmd_port);
  virtual livox_status SendCommand(const Command& command);  
  virtual livox_status SendLoggerCommand(const Command &command);

  static void UpdateLidarCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data);
  void AddDevice(const uint32_t handle);
 private:
  void SetCustomLidar(const uint32_t handle, const uint16_t lidar_cmd_port, const LivoxLidarCfg& lidar_cfg);
  void SetViewLidar(const ViewLidarIpInfo& view_lidar_info);
  livox_status SendCommand(const Command &command, const uint16_t lidar_cmd_port);

  
  bool GetHostInfo(const uint32_t handle, std::string& host_ip, uint16_t& cmd_port);

  void CommandsHandle(TimePoint now);

  void OnCommand(uint32_t handle, const Command &command);
  void OnCommandAck(const uint32_t handle, const Command &command);
  void OnCommandCmd(const uint32_t handle, const uint16_t lidar_port, const Command &command);
  bool IsStatusException(const Command &command);
  void QueryDiagnosisInfo(uint32_t handle);
  void OnLidarInfoChange(const Command &command);
 private:
  std::unique_ptr<CommPort> comm_port_;
  std::mutex device_mutex_;
  std::set<uint32_t> devices_;
  std::map<uint32_t, LivoxLidarCfg> custom_lidars_;
  bool is_view_;
};

}  // namespace livox
} // namespace lidar

#endif  // AVIA2_COMMAND_HANDLER_H_
