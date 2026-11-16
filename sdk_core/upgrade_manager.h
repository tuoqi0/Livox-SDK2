
#ifndef LIVOX_UPGRADE_MANAGER_H_
#define LIVOX_UPGRADE_MANAGER_H_

#include <functional>
#include <memory>
#include <fstream>
#include <vector>
#include <string>

#include "upgrade/livox_lidar_upgrader.h"
#include "upgrade/firmware.h"

namespace livox {
namespace lidar {

class UpgradeManager {
public:
  using OnLivoxLidarUpgradeProgressCallback = 
      std::function<void(uint32_t handle, LivoxLidarUpgradeState state, void *client_data)>;

 private:
  UpgradeManager();
  UpgradeManager(const UpgradeManager& other) = delete;
  UpgradeManager& operator=(const UpgradeManager& other) = delete;
 public:
  typedef std::chrono::steady_clock::time_point TimePoint;
  void Destory();
  ~UpgradeManager() = default;
  static UpgradeManager& GetInstance();

  // Livox lidar upgrade
  bool SetLivoxLidarUpgradeFirmwarePath(const char* firmware_path);
  void SetLivoxLidarUpgradeProgressCallback(OnLivoxLidarUpgradeProgressCallback cb, void* client_data);
  void UpgradeLivoxLidars(const uint32_t* handle, const uint8_t lidar_num);
  void CloseLivoxLidarFirmwareFile();
private:
  Firmware livox_lidar_firmware_;
  OnLivoxLidarUpgradeProgressCallback livox_lidar_info_cb_;
  void *livox_lidar_client_data_;  
};

UpgradeManager &upgrade_manager();

}  // namespace comm
}  // namespace livox

#endif  // LIVOX_UPGRADE_MANAGER_H_
