// Byte layout contracts for protocol records; native configuration types must
// retain the alignment required by their C++ members.
#ifndef LIVOX_SDK_WIRE_LAYOUT_CONTRACT_H_
#define LIVOX_SDK_WIRE_LAYOUT_CONTRACT_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include "comm/define.h"
#include "comm/sdk_protocol.h"
#include "upgrade/firmware.h"

namespace livox {
namespace lidar {
namespace sdk_wire_layout_contract {

#define SDK_WIRE_TYPE(Type, Bytes) \
  static_assert(std::is_standard_layout<Type>::value, #Type " must remain standard layout"); \
  static_assert(std::is_trivially_copyable<Type>::value, #Type " must remain trivially copyable"); \
  static_assert(alignof(Type) == 1, #Type " wire alignment changed"); \
  static_assert(sizeof(Type) == Bytes, #Type " wire size changed")
#define SDK_WIRE_OFFSET(Type, Member, Offset) \
  static_assert(offsetof(Type, Member) == Offset, #Type "." #Member " wire offset changed")
#define SDK_NATIVE_ALIGNMENT(Type, MemberType) \
  static_assert(alignof(Type) >= alignof(MemberType), #Type " requires native member alignment")

// All 19 wire structures formerly sharing define.h's blanket packing scope.
SDK_WIRE_TYPE(DetectionData, 24);
SDK_WIRE_OFFSET(DetectionData, ret_code, 0);
SDK_WIRE_OFFSET(DetectionData, dev_type, 1);
SDK_WIRE_OFFSET(DetectionData, sn, 2);
SDK_WIRE_OFFSET(DetectionData, lidar_ip, 18);
SDK_WIRE_OFFSET(DetectionData, cmd_port, 22);

SDK_WIRE_TYPE(NTPServerIpInfoValue, 4);
SDK_WIRE_OFFSET(NTPServerIpInfoValue, server_ipaddr, 0);

SDK_WIRE_TYPE(LivoxLidarIpInfoValue, 12);
SDK_WIRE_OFFSET(LivoxLidarIpInfoValue, lidar_ipaddr, 0);
SDK_WIRE_OFFSET(LivoxLidarIpInfoValue, lidar_subnet_mask, 4);
SDK_WIRE_OFFSET(LivoxLidarIpInfoValue, lidar_gateway, 8);

SDK_WIRE_TYPE(HostIpInfoValue, 8);
SDK_WIRE_OFFSET(HostIpInfoValue, host_ip, 0);
SDK_WIRE_OFFSET(HostIpInfoValue, host_port, 4);
SDK_WIRE_OFFSET(HostIpInfoValue, lidar_port, 6);

SDK_WIRE_TYPE(EnableDeviceLoggerRequest, 2);
SDK_WIRE_OFFSET(EnableDeviceLoggerRequest, log_type, 0);
SDK_WIRE_OFFSET(EnableDeviceLoggerRequest, enable, 1);

SDK_WIRE_TYPE(DeviceLoggerFilePushRequest, 17);
SDK_WIRE_OFFSET(DeviceLoggerFilePushRequest, log_type, 0);
SDK_WIRE_OFFSET(DeviceLoggerFilePushRequest, file_index, 1);
SDK_WIRE_OFFSET(DeviceLoggerFilePushRequest, file_num, 2);
SDK_WIRE_OFFSET(DeviceLoggerFilePushRequest, flag, 3);
SDK_WIRE_OFFSET(DeviceLoggerFilePushRequest, timestamp, 4);
SDK_WIRE_OFFSET(DeviceLoggerFilePushRequest, rsvd, 8);
SDK_WIRE_OFFSET(DeviceLoggerFilePushRequest, trans_index, 10);
SDK_WIRE_OFFSET(DeviceLoggerFilePushRequest, data_length, 14);
SDK_WIRE_OFFSET(DeviceLoggerFilePushRequest, data, 16);

SDK_WIRE_TYPE(DeviceLoggerFilePushReponse, 7);
SDK_WIRE_OFFSET(DeviceLoggerFilePushReponse, ret_code, 0);
SDK_WIRE_OFFSET(DeviceLoggerFilePushReponse, log_type, 1);
SDK_WIRE_OFFSET(DeviceLoggerFilePushReponse, file_index, 2);
SDK_WIRE_OFFSET(DeviceLoggerFilePushReponse, trans_index, 3);

SDK_WIRE_TYPE(LivoxLidarStartUpgradeRequest, 7);
SDK_WIRE_OFFSET(LivoxLidarStartUpgradeRequest, firmware_type, 0);
SDK_WIRE_OFFSET(LivoxLidarStartUpgradeRequest, encrypt_type, 1);
SDK_WIRE_OFFSET(LivoxLidarStartUpgradeRequest, firmware_length, 2);
SDK_WIRE_OFFSET(LivoxLidarStartUpgradeRequest, dev_type, 6);

SDK_WIRE_TYPE(LivoxLidarStartUpgradeRequestV3, 51);
SDK_WIRE_OFFSET(LivoxLidarStartUpgradeRequestV3, firmware_type, 0);
SDK_WIRE_OFFSET(LivoxLidarStartUpgradeRequestV3, encrypt_type, 1);
SDK_WIRE_OFFSET(LivoxLidarStartUpgradeRequestV3, firmware_length, 2);
SDK_WIRE_OFFSET(LivoxLidarStartUpgradeRequestV3, dev_type, 6);
SDK_WIRE_OFFSET(LivoxLidarStartUpgradeRequestV3, firmware_version, 7);
SDK_WIRE_OFFSET(LivoxLidarStartUpgradeRequestV3, firmware_buildtime, 11);
SDK_WIRE_OFFSET(LivoxLidarStartUpgradeRequestV3, hw_whitelist, 19);

SDK_WIRE_TYPE(LivoxLidarStartUpgradeResponse, 1);
SDK_WIRE_OFFSET(LivoxLidarStartUpgradeResponse, ret_code, 0);

SDK_WIRE_TYPE(LivoxLidarXferFirmwareResquest, 13);
SDK_WIRE_OFFSET(LivoxLidarXferFirmwareResquest, offset, 0);
SDK_WIRE_OFFSET(LivoxLidarXferFirmwareResquest, length, 4);
SDK_WIRE_OFFSET(LivoxLidarXferFirmwareResquest, encrypt_type, 8);
SDK_WIRE_OFFSET(LivoxLidarXferFirmwareResquest, rsvd, 9);
SDK_WIRE_OFFSET(LivoxLidarXferFirmwareResquest, data, 12);

SDK_WIRE_TYPE(LivoxLidarXferFirmwareResponse, 9);
SDK_WIRE_OFFSET(LivoxLidarXferFirmwareResponse, ret_code, 0);
SDK_WIRE_OFFSET(LivoxLidarXferFirmwareResponse, offset, 1);
SDK_WIRE_OFFSET(LivoxLidarXferFirmwareResponse, length, 5);

SDK_WIRE_TYPE(LivoxLidarCompleteXferFirmwareResquest, 3);
SDK_WIRE_OFFSET(LivoxLidarCompleteXferFirmwareResquest, checksum_type, 0);
SDK_WIRE_OFFSET(LivoxLidarCompleteXferFirmwareResquest, checksum_length, 1);
SDK_WIRE_OFFSET(LivoxLidarCompleteXferFirmwareResquest, checksum, 2);

SDK_WIRE_TYPE(LivoxLidarCompleteXferFirmwareResponse, 1);
SDK_WIRE_OFFSET(LivoxLidarCompleteXferFirmwareResponse, ret_code, 0);

SDK_WIRE_TYPE(LivoxLidarGetUpgradeProgressResponse, 2);
SDK_WIRE_OFFSET(LivoxLidarGetUpgradeProgressResponse, ret_code, 0);
SDK_WIRE_OFFSET(LivoxLidarGetUpgradeProgressResponse, progress, 1);

SDK_WIRE_TYPE(LivoxLidarRequestFirmwareInfoResponse, 4);
SDK_WIRE_OFFSET(LivoxLidarRequestFirmwareInfoResponse, ret_code, 0);
SDK_WIRE_OFFSET(LivoxLidarRequestFirmwareInfoResponse, length, 1);
SDK_WIRE_OFFSET(LivoxLidarRequestFirmwareInfoResponse, info, 3);

SDK_WIRE_TYPE(LivoxLidarDebugPointCloudFileHeader, 128);
SDK_WIRE_OFFSET(LivoxLidarDebugPointCloudFileHeader, file_ver, 0);
SDK_WIRE_OFFSET(LivoxLidarDebugPointCloudFileHeader, dev_type, 1);
SDK_WIRE_OFFSET(LivoxLidarDebugPointCloudFileHeader, data_type, 2);
SDK_WIRE_OFFSET(LivoxLidarDebugPointCloudFileHeader, sn, 3);
SDK_WIRE_OFFSET(LivoxLidarDebugPointCloudFileHeader, rsvd, 19);
SDK_WIRE_OFFSET(LivoxLidarDebugPointCloudFileHeader, crc16, 126);

SDK_WIRE_TYPE(LivoxLidarDebugPointCloudRequest, 9);
SDK_WIRE_OFFSET(LivoxLidarDebugPointCloudRequest, enable, 0);
SDK_WIRE_OFFSET(LivoxLidarDebugPointCloudRequest, host_ip_addr, 1);
SDK_WIRE_OFFSET(LivoxLidarDebugPointCloudRequest, host_port, 5);
SDK_WIRE_OFFSET(LivoxLidarDebugPointCloudRequest, bandwidth, 7);

SDK_WIRE_TYPE(LivoxLidarRmcSyncTimeRequest, 9);
SDK_WIRE_OFFSET(LivoxLidarRmcSyncTimeRequest, type, 0);
SDK_WIRE_OFFSET(LivoxLidarRmcSyncTimeRequest, ns, 1);

// Adjacent protocol/file headers remain unchanged and retain their wire layouts.
SDK_WIRE_TYPE(SdkPreamble, 24);
SDK_WIRE_OFFSET(SdkPreamble, sof, 0);
SDK_WIRE_OFFSET(SdkPreamble, version, 1);
SDK_WIRE_OFFSET(SdkPreamble, length, 2);
SDK_WIRE_OFFSET(SdkPreamble, seq_num, 4);
SDK_WIRE_OFFSET(SdkPreamble, cmd_id, 8);
SDK_WIRE_OFFSET(SdkPreamble, cmd_type, 10);
SDK_WIRE_OFFSET(SdkPreamble, sender_type, 11);
SDK_WIRE_OFFSET(SdkPreamble, rsvd, 12);
SDK_WIRE_OFFSET(SdkPreamble, crc16_h, 18);
SDK_WIRE_OFFSET(SdkPreamble, crc32_d, 20);

SDK_WIRE_TYPE(SdkPacket, 25);
SDK_WIRE_OFFSET(SdkPacket, sof, 0);
SDK_WIRE_OFFSET(SdkPacket, version, 1);
SDK_WIRE_OFFSET(SdkPacket, length, 2);
SDK_WIRE_OFFSET(SdkPacket, seq_num, 4);
SDK_WIRE_OFFSET(SdkPacket, cmd_id, 8);
SDK_WIRE_OFFSET(SdkPacket, cmd_type, 10);
SDK_WIRE_OFFSET(SdkPacket, sender_type, 11);
SDK_WIRE_OFFSET(SdkPacket, rsvd, 12);
SDK_WIRE_OFFSET(SdkPacket, crc16_h, 18);
SDK_WIRE_OFFSET(SdkPacket, crc32_d, 20);
SDK_WIRE_OFFSET(SdkPacket, data, 24);

SDK_WIRE_TYPE(LivoxEncryptFirmwareHeader, 286);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareHeader, file_version, 0);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareHeader, firmware_version, 4);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareHeader, firmware_length, 8);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareHeader, firmware_type, 12);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareHeader, device_type, 13);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareHeader, encrypt_type, 14);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareHeader, rsvd, 15);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareHeader, checksum_type, 17);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareHeader, checksum_length, 18);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareHeader, checksum, 20);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareHeader, hw_whitelist, 148);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareHeader, modify_time, 276);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareHeader, header_checksum, 284);

SDK_WIRE_TYPE(LivoxEncryptFirmwareTail, 16);
SDK_WIRE_OFFSET(LivoxEncryptFirmwareTail, overall_signature, 0);

// Native C++ configurations are not wire records. Do not pin std::string/vector
// sizes or use offsetof on their potentially non-standard-layout aggregates.
// These static alignment requirements complement actual full-UBSan construction;
// they alone do not prove every interior member on every C++ library ABI.
SDK_NATIVE_ALIGNMENT(LivoxLidarNetInfo, std::string);
SDK_NATIVE_ALIGNMENT(LivoxLidarNetInfo, std::uint16_t);
SDK_NATIVE_ALIGNMENT(HostNetInfo, std::string);
SDK_NATIVE_ALIGNMENT(HostNetInfo, std::uint16_t);
SDK_NATIVE_ALIGNMENT(GeneralCfgInfo, std::vector<std::uint16_t>);
SDK_NATIVE_ALIGNMENT(LivoxLidarCfg, LivoxLidarNetInfo);
SDK_NATIVE_ALIGNMENT(LivoxLidarCfg, HostNetInfo);
SDK_NATIVE_ALIGNMENT(LivoxLidarCfg, GeneralCfgInfo);
SDK_NATIVE_ALIGNMENT(LivoxLidarLoggerCfg, std::string);
SDK_NATIVE_ALIGNMENT(LivoxLidarLoggerCfg, std::uint64_t);
SDK_NATIVE_ALIGNMENT(LivoxLidarSdkFrameworkCfg, bool);
SDK_NATIVE_ALIGNMENT(ViewDevice, std::uint32_t);
SDK_NATIVE_ALIGNMENT(ViewDevice, std::atomic<bool>);
SDK_NATIVE_ALIGNMENT(ViewLidarIpInfo, std::string);
SDK_NATIVE_ALIGNMENT(ViewLidarIpInfo, std::uint32_t);
SDK_NATIVE_ALIGNMENT(ViewLidarIpInfo, std::uint16_t);
SDK_NATIVE_ALIGNMENT(LidarDeviceInfo, std::string);
SDK_NATIVE_ALIGNMENT(LidarDeviceInfo, std::uint16_t);

// A private wire header must not leave the including translation unit packed.
struct FollowingNativeRecord {
  std::uint8_t prefix;
  std::uint64_t payload;
};
static_assert(alignof(FollowingNativeRecord) >= alignof(std::uint64_t),
              "wire header leaked packing to native declarations");
static_assert(offsetof(FollowingNativeRecord, payload) % alignof(std::uint64_t) == 0,
              "wire header leaked packing to the next native member");

#undef SDK_NATIVE_ALIGNMENT
#undef SDK_WIRE_OFFSET
#undef SDK_WIRE_TYPE

}  // namespace sdk_wire_layout_contract
}  // namespace lidar
}  // namespace livox
#endif
