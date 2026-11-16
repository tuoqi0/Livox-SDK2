
#ifndef LIVOX_LIDAR_UPGRADER_H_
#define LIVOX_LIDAR_UPGRADER_H_

#include <fstream>
#include <ios>
#include <memory>
#include <thread>
#include <functional>

#include "firmware.h"
#include "../comm/define.h"
#include "livox_lidar_def.h"

namespace livox {
namespace lidar {

static const uint8_t EraseFirmware = 0x34;

class LivoxLidarUpgrader {
 public:  
  using UpgradeProgressCallback = std::function<void(uint32_t handle, LivoxLidarUpgradeState state)>;

  LivoxLidarUpgrader(const Firmware& firmware, const uint32_t handle);
  ~LivoxLidarUpgrader();

  void AddUpgradeProgressObserver(UpgradeProgressCallback observer);
  bool StartUpgradeLivoxLidar();

  livox_status StartUpgrade();
  livox_status XferFirmware();
  livox_status CompleteXferFirmware();
  livox_status GetUpgradeProgress();
  livox_status UpgradeComplete();

  static void StartUpgradeResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarStartUpgradeResponse* response, void* client_data);
  static void XferFirmwareResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarXferFirmwareResponse* response, void* client_data);
  static void CompleteXferFirmwareResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarCompleteXferFirmwareResponse* response, void* client_data);
  static void GetProgressResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarGetUpgradeProgressResponse* response, void* client_data);
  static void UpgradeCompleteResponseHandler(livox_status status, uint32_t handle,
      LivoxLidarRebootResponse* response, void* client_data);

  int32_t LivoxLidarFsmStateChange(LivoxLidarFsmEvent event);
  void FsmEventHandler(LivoxLidarFsmEvent event, uint8_t progress);

  bool IsUpgradeComplete() { return (fsm_state_ == kLivoxLidarUpgradeIdle); }
  bool IsUpgradeError() { return (fsm_state_ == kLivoxLidarUpgradeTimeout) || (fsm_state_ == kLivoxLidarUpgradeErr); }

 private:
  const Firmware& firmware_;
  uint32_t read_offset_;
  uint32_t read_length_;
  uint32_t handle_;
  uint32_t fsm_state_;

  uint32_t upgrade_error_;
  uint8_t progress_;

  uint32_t try_count_;
  std::shared_ptr<std::thread> upgrade_thread_;
  UpgradeProgressCallback observer_;

};

} // namespace comm
} // namespace LIVOX_LIDAR_UPGRADER_H_

#endif
