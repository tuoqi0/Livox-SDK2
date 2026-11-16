#ifndef LIVOX_BUILD_REQUEST_H_
#define LIVOX_BUILD_REQUEST_H_

#include <memory>
#include <map>
#include <list>
#include <string>
#include <algorithm>

#include <string.h>

#include "base/io_loop.h"
#include "comm/comm_port.h"

#include "livox_lidar_def.h"
#include "comm/define.h"

#include <memory>

namespace livox {
namespace lidar {


class BuildRequest {
 public:
  static bool BuildUpdateViewLidarCfgRequest(const ViewLidarIpInfo& view_lidar_info, uint8_t* req_buf, uint16_t& req_len);
  static bool BuildUpdateLidarCfgRequest(const LivoxLidarCfg& lidar_cfg, uint8_t* req_buf, uint16_t& req_len);
  static bool BuildUpdateMid360LidarCfgRequest(const LivoxLidarCfg& lidar_cfg, uint8_t* req_buf, uint16_t& req_len);
  static bool BuildUpdateAvia2LidarCfgRequest(const LivoxLidarCfg& lidar_cfg, uint8_t* req_buf, uint16_t& req_len);
  static bool BuildSetLidarIPInfoRequest(const LivoxLidarIpInfo& ip_config, uint8_t* req_buf, uint16_t& req_len);
  static bool BuildSetHostStateInfoIPCfgRequest(const HostStateInfoIpInfo& host_state_info_ipcfg, uint8_t* req_buf, uint16_t& req_len);
  static bool BuildSetHostPointDataIPInfoRequest(const HostPointIPInfo& lidar_ip_config, uint8_t* req_buf, uint16_t& req_len);
  static bool BuildSetHostImuDataIPInfoRequest(const HostImuDataIPInfo& host_imu_ipcfg, uint8_t* req_buf, uint16_t& req_len);
  static bool IpToU8(const std::string& src, const std::string& seq, std::vector<uint8_t>& result);
  static bool BuildSetNTPserverIPInfoRequest(const NTPServerIpInfo& server_ip_config, uint8_t* req_buf, uint16_t& req_len);
 private:
  static bool InitLidarIpinfoVal(const LivoxLidarIpInfo& lidar_ip_config, LivoxLidarIpInfoValue* lidar_ipinfo_val_ptr);
  static bool InitMulticastHostIpAddr(const std::string& multicast_ip, HostIpInfoValue* host_ipinfo_val_ptr);
  static bool InitHostIpAddr(const std::string& host_ip, HostIpInfoValue* host_ipinfo_val_ptr);
  static bool InitServerIpinfoVal(const NTPServerIpInfo& ntp_server_ip_config, NTPServerIpInfoValue* ntp_server_ip_val_ptr);
};

} // namespace direct
} // namespace livox

# endif // LIVOX_BUILD_REQUEST_H_
