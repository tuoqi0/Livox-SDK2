
#ifndef LIVOX_PARSE_CFG_FILE_H_
#define LIVOX_PARSE_CFG_FILE_H_

#include "livox_lidar_def.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/stringbuffer.h"

#include "comm/define.h"

#include <iostream>
#include <string>
#include <memory>
#include <vector>

namespace livox {
namespace lidar {

class ParseCfgFile {
 public:
  explicit ParseCfgFile(const std::string& path);  
  bool Parse(std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
             std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr,
             std::shared_ptr<LivoxLidarLoggerCfg>& lidar_logger_cfg_ptr,
             std::shared_ptr<LivoxLidarSdkFrameworkCfg>& sdk_framework_cfg_ptr
             );
 private:
  bool ParseLidarCfg(const rapidjson::Value &object,
                     const uint8_t& device_type,
                     std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
                     std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr);
  bool ParseNewLidarCfg(const rapidjson::Value &object,
                     const uint8_t& device_type,
                     std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr,
                     std::shared_ptr<std::vector<LivoxLidarCfg>>& custom_lidars_cfg_ptr);
  bool ParseOldLidarCfg(const rapidjson::Value &object,
                        const uint8_t& device_type,
                        std::shared_ptr<std::vector<LivoxLidarCfg>>& lidars_cfg_ptr);
  bool ParseTypeLidarCfg(const rapidjson::Value &object, const rapidjson::Value &host_net_info_object, const uint8_t& device_type, LivoxLidarCfg& lidar_cfg);
  bool ParseLidarNetInfo(const rapidjson::Value &object, LivoxLidarNetInfo& lidar_net_info);
  bool ParseHostNetInfo(const rapidjson::Value &host_net_info_object, HostNetInfo& host_net_info);
  bool ParseGeneralCfgInfo(const rapidjson::Value &object, GeneralCfgInfo& general_cfg_info);
 private:
  const std::string path_;
};

} // namesace lidar
} // namespace livox

#endif // LIVOX_PARSE_CFG_FILE_H_

