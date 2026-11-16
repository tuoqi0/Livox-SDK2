
#ifndef PARSE_LIDAR_STATE_INFO_H_
#define PARSE_LIDAR_STATE_INFO_H_

#include <memory>
#include <map>
#include <mutex>
#include <set>

#include "comm/define.h"
#include "comm/protocol.h"

#include "livox_lidar_def.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/prettywriter.h"

namespace livox {

namespace lidar {

class ParseLidarStateInfo {
 public:
  static bool Parse(const CommPacket& packet, std::string& info);
 private:
  static bool ParseStateInfo(const CommPacket& packet, DirectLidarStateInfo& info, std::set<ParamKeyName>& key_mask);
  static void ParseLidarIpAddr(const CommPacket& packet, uint16_t off, DirectLidarStateInfo& info);
  static void ParseStateInfoHostIPCfg(const CommPacket& packet, uint16_t off, DirectLidarStateInfo& info);
  static void ParsePointCloudHostIpCfg(const CommPacket& packet, uint16_t off, DirectLidarStateInfo& info);
  static void ParseImuDataHostIpCfg(const CommPacket& packet, uint16_t off, DirectLidarStateInfo& info);
  static void ParseIpCfg(const CommPacket& packet, uint16_t off, LivoxIpCfg& cfg);
  static void LivoxLidarStateInfoToJson(const DirectLidarStateInfo& info, const std::set<ParamKeyName>& key_mask, std::string& lidar_info);
};

} // namespace livox
} // namespace lidar

# endif // PARSE_LIDAR_STATE_INFO_H_




















