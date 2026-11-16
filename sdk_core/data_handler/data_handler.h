
#ifndef LIVOX_DATA_HANDLER_H_
#define LIVOX_DATA_HANDLER_H_

#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include "comm/define.h"
#include "base/io_loop.h"

namespace livox {
namespace lidar {

class DataHandler : public noncopyable {
 private:
  DataHandler();
  DataHandler(const DataHandler& other) = delete;
  DataHandler& operator=(const DataHandler& other) = delete;
 public:
  void Destory();
  ~DataHandler();
  static DataHandler& GetInstance();

  bool Init();

  void Handle(const uint8_t dev_type, const uint32_t handle, uint8_t *buf, uint32_t buf_size);

  uint16_t AddPointCloudObserver(const DataCallback &cb, void *client_data);
  void RemovePointCloudObserver(uint16_t id);

  void SetPointDataCallback(const DataCallback& cb, void *client_data);
  void SetImuDataCallback(const DataCallback& cb, void* client_data);

 private:
  uint16_t GenerateObserverId();
 private:
  DataCallback point_data_callbacks_;
  void* point_client_data_;

  DataCallback imu_data_callbacks_;
  void* imu_client_data_;

  std::map<uint16_t, std::pair<DataCallback, void*>> observers_;
  std::mutex mutex_;
};

} // namespace lidar
}  // namespace livox

#endif  // LIVOX_DATA_HANDLER_H_















