#include <robot_core/test/doctest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "base/logging.h"
#include "data_handler/data_handler.h"
#include "livox_lidar_def.h"

namespace {
using livox::lidar::DataHandler;

std::size_t observed_packets = 0;

void observePacket(
  const uint32_t, const uint8_t, LivoxLidarEthernetPacket *, void *)
{
  ++observed_packets;
}

std::vector<uint8_t> makePacket(
  const uint8_t data_type, const uint16_t dot_num, const std::size_t payload_size)
{
  const std::size_t header_size = offsetof(LivoxLidarEthernetPacket, data);
  std::vector<uint8_t> bytes(header_size + payload_size, 0);
  auto * packet = reinterpret_cast<LivoxLidarEthernetPacket *>(bytes.data());
  packet->length = static_cast<uint16_t>(bytes.size());
  packet->dot_num = dot_num;
  packet->data_type = data_type;
  return bytes;
}

}  // namespace

TEST_CASE("SDK drops malformed lidar packets before dispatch")
{
  InitLogger();
  DataHandler & handler = DataHandler::GetInstance();
  handler.Destory();
  observed_packets = 0;
  const uint16_t observer_id = handler.AddPointCloudObserver(observePacket, nullptr);
  const std::size_t point_size = sizeof(LivoxLidarCartesianHighRawPoint);
  const std::size_t header_size = offsetof(LivoxLidarEthernetPacket, data);

  handler.Handle(0, 1, nullptr, 0);
  std::vector<uint8_t> short_header(header_size - 1, 0);
  handler.Handle(0, 1, short_header.data(), short_header.size());

  auto length_overrun = makePacket(kLivoxLidarCartesianCoordinateHighData, 1, point_size);
  auto * length_packet = reinterpret_cast<LivoxLidarEthernetPacket *>(length_overrun.data());
  ++length_packet->length;
  handler.Handle(0, 1, length_overrun.data(), length_overrun.size());

  auto short_payload = makePacket(kLivoxLidarCartesianCoordinateHighData, 1, point_size - 1);
  handler.Handle(0, 1, short_payload.data(), short_payload.size());

  auto unknown_type = makePacket(0xff, 1, point_size);
  handler.Handle(0, 1, unknown_type.data(), unknown_type.size());

  auto zero_dot = makePacket(kLivoxLidarCartesianCoordinateHighData, 0, point_size);
  handler.Handle(0, 1, zero_dot.data(), zero_dot.size());
  CHECK(observed_packets == 0);

  auto valid = makePacket(kLivoxLidarCartesianCoordinateHighData, 1, point_size);
  handler.Handle(0, 1, valid.data(), valid.size());
  CHECK(observed_packets == 1);

  auto imu_zero_dot = makePacket(kLivoxLidarImuData, 0, sizeof(LivoxLidarImuRawPoint));
  handler.Handle(0, 1, imu_zero_dot.data(), imu_zero_dot.size());
  CHECK(observed_packets == 2);

  handler.RemovePointCloudObserver(observer_id);
  handler.Destory();
  UninitLogger();
}
