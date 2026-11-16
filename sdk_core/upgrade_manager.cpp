
#include "upgrade_manager.h"

namespace livox {
namespace lidar {

UpgradeManager& UpgradeManager::GetInstance() {
  static UpgradeManager manager;
  return manager;
}

UpgradeManager::UpgradeManager()
    : livox_lidar_info_cb_(nullptr),
      livox_lidar_client_data_(nullptr) {
}

bool UpgradeManager::SetLivoxLidarUpgradeFirmwarePath(const char* firmware_path) {
  if (!livox_lidar_firmware_.Open(firmware_path)) {
    printf("Open firmware_path fail\r\n");
    return false;
  }
  return true;
}

void UpgradeManager::SetLivoxLidarUpgradeProgressCallback(OnLivoxLidarUpgradeProgressCallback cb, void* client_data) {
  livox_lidar_info_cb_ = cb;
  livox_lidar_client_data_ = client_data;
}

void UpgradeManager::UpgradeLivoxLidars(const uint32_t* handle, const uint8_t lidar_num) {
  std::vector<LivoxLidarUpgrader> upgrader_vec;
  upgrader_vec.reserve(lidar_num);
  for (size_t i = 0; i < lidar_num; ++i) {   
    LivoxLidarUpgrader upgrader(livox_lidar_firmware_, handle[i]);

    OnLivoxLidarUpgradeProgressCallback cb = livox_lidar_info_cb_;
    void* client_data = livox_lidar_client_data_;
    upgrader.AddUpgradeProgressObserver([cb, client_data](uint32_t handle, LivoxLidarUpgradeState state) {
      if (cb) {
        cb(handle, state, client_data);
      }
    });
 
    upgrader_vec.emplace_back(std::move(upgrader));
  }

  for (size_t i = 0; i < upgrader_vec.size(); ++i) {
    LivoxLidarUpgrader& upgrader = upgrader_vec[i];
    upgrader.StartUpgradeLivoxLidar();
  }

  CloseLivoxLidarFirmwareFile();
}

void UpgradeManager::CloseLivoxLidarFirmwareFile() {
  livox_lidar_firmware_.Close();
}

}  // namespace comm
}  // namespace livox
