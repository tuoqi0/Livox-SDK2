
#ifndef LIVOX_PARAMS_CHECK_H_
#define LIVOX_PARAMS_CHECK_H_

#include "livox_lidar_def.h"

#include "comm/define.h"

#include <iostream>
#include <string>
#include <memory>
#include <vector>

#include "command_handler/build_request.h"

namespace livox {
namespace lidar {

class ParamsCheck {
 public:
  ParamsCheck(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
      std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr);
  bool Check();
 private:
  bool CheckLidarIp();
  bool CheckLidarMulticastIp();
  void CheckLidarPort();
  void CheckPort(const uint8_t dev_type, LivoxLidarNetInfo& lidar_net_info);
 private:
  std::shared_ptr<std::vector<LivoxLidarCfg>> lidars_cfg_ptr_;
  std::shared_ptr<std::vector<LivoxLidarCfg>> custom_lidars_cfg_ptr_;
};

} // namespace lidar
} // namespace livox

#endif // LIVOX_PARSE_CFG_FILE_H_

