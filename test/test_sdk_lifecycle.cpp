// Linux transport fault injection against the actual SDK shared library.
// No API lifecycle, IOThread, epoll loop or cleanup implementation is copied.
#include <robot_core/test/doctest.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <dirent.h>
#include <fcntl.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>
#include "livox_lidar_api.h"
#include "base/logging.h"
#define private public
#include "device_manager.h"
#include "data_handler/data_handler.h"
#include "command_handler/general_command_handler.h"
#include "logger_handler/logger_manager.h"
#undef private

namespace {
using namespace livox::lidar;
unsigned calls = 0;
unsigned fail_call = 0;
unsigned threads_at_failure = 0;
std::vector<int> created_fds;
std::atomic<unsigned> prohibited_socket_calls{0};
std::atomic<unsigned> swallowed_sendto_calls{0};

size_t directoryCount(const char* path) {
  DIR* dir = opendir(path);
  REQUIRE(dir != nullptr);
  size_t count = 0;
  while (const dirent* entry = readdir(dir))
    if (entry->d_name[0] != '.') ++count;
  closedir(dir);
  return count;
}

unsigned liveIoThreads() {
  const auto& manager = DeviceManager::GetInstance();
  unsigned count = 0;
  for (const auto& io : {manager.detection_io_thread_, manager.cmd_io_thread_,
                          manager.data_io_thread_}) {
    if (io && io->thread_ && io->thread_->joinable() && io->loop_ && !io->IsQuit())
      ++count;
  }
  return count;
}

bool callbacksCleared() {
  const auto& command = GeneralCommandHandler::GetInstance();
  const auto& data = DataHandler::GetInstance();
  return !command.livox_lidar_info_change_cb_ &&
      !command.livox_lidar_info_change_client_data_ && data.observers_.empty() &&
      !data.point_data_callbacks_ && !data.imu_data_callbacks_ &&
      !data.point_client_data_ && !data.imu_client_data_;
}

bool managerCleared() {
  const auto& m = DeviceManager::GetInstance();
  return !m.detection_thread_ && !m.detection_io_thread_ && !m.cmd_io_thread_ &&
      !m.data_io_thread_ && !m.comm_port_ && !m.lidars_cfg_ptr_ &&
      !m.custom_lidars_cfg_ptr_ && m.socket_vec_.empty() &&
      m.vec_broadcast_socket_.empty() && m.channel_info_.empty() &&
      m.command_channel_.empty() && m.data_channel_.empty() &&
      m.custom_command_channel_.empty() && m.type_lidars_cfg_map_.empty() &&
      m.custom_lidars_cfg_map_.empty();
}

unsigned openInjectedFds() {
  unsigned count = 0;
  for (int fd : created_fds)
    if (fcntl(fd, F_GETFD) != -1) ++count;
  return count;
}

void rescueAfterObservation() {
  // Diagnostic cleanup only, after recording all observations. Quiesce real
  // threads before old Destory clears fields they may still read. Keep the
  // joined std::thread objects present: old IOThread destruction calls Join
  // again and its legacy nonjoinable branch requires a nonnull owner.
  auto& manager = DeviceManager::GetInstance();
  const std::array<std::shared_ptr<IOThread>, 3> io_threads{{
      manager.detection_io_thread_, manager.cmd_io_thread_, manager.data_io_thread_}};
  for (const auto& io : io_threads)
    if (io) io->quit_.store(true);
  for (const auto& io : io_threads)
    if (io && io->thread_ && io->thread_->joinable()) io->thread_->join();
  manager.is_stop_detection_.store(true);
  if (manager.detection_thread_ && manager.detection_thread_->joinable())
    manager.detection_thread_->join();
  manager.detection_thread_.reset();
  manager.Destory();
  manager.detection_io_thread_.reset();
  manager.cmd_io_thread_.reset();
  manager.data_io_thread_.reset();
  for (int fd : created_fds)
    if (fcntl(fd, F_GETFD) != -1) close(fd);
  manager.detection_socket_ = -1;
  manager.detection_broadcast_socket_ = -1;
  DataHandler::GetInstance().Destory();
  GeneralCommandHandler::GetInstance().Destory();
  LoggerManager::GetInstance().Destory();
  UninitLogger();
}

void infoCallback(uint32_t, const LivoxLidarInfo*, void*) {}
void pointCallback(uint32_t, uint8_t, LivoxLidarEthernetPacket*, void*) {}

class ConfigFile {
 public:
  ConfigFile() {
    constexpr const char* json = R"json({
      "MID360": {
        "lidar_net_info": {"cmd_data_port":56100,"push_msg_port":56200,
          "point_data_port":56300,"imu_data_port":56400,"log_data_port":56500},
        "host_net_info": [{"lidar_ip":["127.0.0.81"],"host_ip":"127.0.0.1",
          "cmd_data_port":56101,"push_msg_port":56201,"point_data_port":56301,
          "imu_data_port":56401,"log_data_port":56501}]
      }
    })json";
    const int fd = mkstemp(path_);
    REQUIRE(fd >= 0);
    FILE* file = fdopen(fd, "wb");
    if (!file) close(fd);
    REQUIRE(file != nullptr);
    const size_t written = std::fwrite(json, 1, std::strlen(json), file);
    const int closed = std::fclose(file);
    REQUIRE(written == std::strlen(json));
    REQUIRE(closed == 0);
  }
  ~ConfigFile() { std::remove(path_); }
  const char* path() const { return path_; }
 private:
  char path_[64] = "/tmp/livox-sdk-partial-config-XXXXXX";
};

void runFailure(unsigned failure, bool use_config) {
  ConfigFile config;
  // Prime only the real inert singleton objects, before measuring resources.
  (void)DeviceManager::GetInstance();
  (void)GeneralCommandHandler::GetInstance();
  (void)DataHandler::GetInstance();
  (void)LoggerManager::GetInstance();
  DisableLivoxSdkConsoleLogger();
  const size_t original_tasks = directoryCount("/proc/self/task");
  const size_t original_fds = directoryCount("/proc/self/fd");
  calls = 0;
  fail_call = failure;
  threads_at_failure = 0;
  created_fds.clear();
  SetLivoxLidarInfoChangeCallback(infoCallback, &calls);
  LivoxLidarAddPointCloudObserver(pointCallback, &calls);
  SetLivoxLidarPointCloudCallBack(pointCallback, &calls);
  SetLivoxLidarImuDataCallback(pointCallback, &calls);

  const bool initialized = use_config ? LivoxLidarSdkInit(config.path()) :
      LivoxLidarSdkInit(nullptr, "127.0.0.1", nullptr);
  const unsigned live_after_return = liveIoThreads();
  const unsigned open_after_return = openInjectedFds();
  const size_t tasks_after_return = directoryCount("/proc/self/task");
  const size_t fds_after_return = directoryCount("/proc/self/fd");
  const bool manager_clean = managerCleared();
  const bool callbacks_clean = callbacksCleared();
  LivoxLidarSdkUninit();
  const bool cleanup_needed = !managerCleared() || !callbacksCleared() || openInjectedFds();
  if (cleanup_needed) rescueAfterObservation();
  const size_t rescued_tasks = directoryCount("/proc/self/task");
  const size_t rescued_fds = directoryCount("/proc/self/fd");
  std::printf("SDK_PARTIAL fail_call=%u calls=%u initialized=%d created=%zu "
              "threads_at_failure=%u live_after=%u open_after=%u tasks=%zu/%zu "
              "fds=%zu/%zu manager_clean=%d callbacks_clean=%d rescue=%d "
              "rescue_tasks=%zu rescue_fds=%zu socket_denied=%u sendto_swallowed=%u\n",
      failure,calls,int(initialized),created_fds.size(),threads_at_failure,
      live_after_return,open_after_return,tasks_after_return,original_tasks,
      fds_after_return,original_fds,int(manager_clean),int(callbacks_clean),
      int(cleanup_needed),rescued_tasks,rescued_fds,
      prohibited_socket_calls.load(),swallowed_sendto_calls.load());
  CHECK_FALSE(initialized);
  CHECK(calls == failure);
  CHECK(created_fds.size() == failure - 1);
  CHECK(threads_at_failure == 3);
  CHECK(live_after_return == 0);
  CHECK(open_after_return == 0);
  CHECK(tasks_after_return == original_tasks);
  CHECK(fds_after_return == original_fds);
  CHECK(manager_clean);
  CHECK(callbacks_clean);
  CHECK_FALSE(cleanup_needed);
  CHECK(rescued_tasks == original_tasks);
  CHECK(rescued_fds == original_fds);
  CHECK(prohibited_socket_calls.load() == 0);
  CHECK(swallowed_sendto_calls.load() == 0);
}

void runSuccessAndRetry() {
  ConfigFile config;
  (void)DeviceManager::GetInstance();
  (void)GeneralCommandHandler::GetInstance();
  (void)DataHandler::GetInstance();
  (void)LoggerManager::GetInstance();
  DisableLivoxSdkConsoleLogger();
  const size_t original_tasks = directoryCount("/proc/self/task");
  const size_t original_fds = directoryCount("/proc/self/fd");
  fail_call = 0;
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    calls = 0;
    created_fds.clear();
    const unsigned sent_before = swallowed_sendto_calls.load();
    SetLivoxLidarInfoChangeCallback(infoCallback, &calls);
    LivoxLidarAddPointCloudObserver(pointCallback, &calls);
    const bool initialized = LivoxLidarSdkInit(config.path());
    const unsigned live_before_stop = liveIoThreads();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (initialized && swallowed_sendto_calls.load() == sent_before &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const unsigned sent_this_generation = swallowed_sendto_calls.load() - sent_before;
    const auto& manager = DeviceManager::GetInstance();
    const bool detection_started = manager.detection_thread_ &&
                                   manager.detection_thread_->joinable();
    LivoxLidarSdkUninit();
    const unsigned live_after_stop = liveIoThreads();
    const unsigned open_after_stop = openInjectedFds();
    const bool manager_clean = managerCleared();
    const bool callbacks_clean = callbacksCleared();
    const size_t tasks_after_stop = directoryCount("/proc/self/task");
    const size_t fds_after_stop = directoryCount("/proc/self/fd");
    const bool cleanup_needed = !manager_clean || !callbacks_clean || open_after_stop;
    if (cleanup_needed) rescueAfterObservation();
    const size_t rescued_tasks = directoryCount("/proc/self/task");
    const size_t rescued_fds = directoryCount("/proc/self/fd");
    std::printf("SDK_SUCCESS attempt=%u initialized=%d created=%zu live_before=%u "
                "detection_joinable=%d detection_sends=%u live_after=%u open_after=%u "
                "tasks=%zu/%zu fds=%zu/%zu manager_clean=%d callbacks_clean=%d "
                "rescue=%d rescue_tasks=%zu rescue_fds=%zu socket_denied=%u\n",
        attempt,int(initialized),created_fds.size(),live_before_stop,
        int(detection_started),sent_this_generation,live_after_stop,open_after_stop,
        tasks_after_stop,original_tasks,fds_after_stop,original_fds,int(manager_clean),
        int(callbacks_clean),int(cleanup_needed),rescued_tasks,rescued_fds,
        prohibited_socket_calls.load());
    CAPTURE(attempt);
    CHECK(initialized);
    CHECK(calls >= 7);
    CHECK(live_before_stop == 3);
    CHECK(detection_started);
    CHECK(sent_this_generation >= 1);
    CHECK(live_after_stop == 0);
    CHECK(open_after_stop == 0);
    CHECK(manager_clean);
    CHECK(callbacks_clean);
    CHECK(tasks_after_stop == original_tasks);
    CHECK(fds_after_stop == original_fds);
    CHECK_FALSE(cleanup_needed);
    CHECK(rescued_tasks == original_tasks);
    CHECK(rescued_fds == original_fds);
    CHECK(prohibited_socket_calls.load() == 0);
  }
}
}

// Export this exact C++ symbol from the test executable. Shared-SDK calls must
// resolve through its PLT to this definition; audit .dynsym and relocations.
namespace livox { namespace lidar { namespace util {
socket_t CreateSocket(uint16_t, bool, bool, bool,
                       const std::string, const std::string) {
  ++calls;
  if (calls == fail_call) {
    threads_at_failure = liveIoThreads();
    errno = EMFILE;
    return -1;
  }
  const int fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (fd >= 0) created_fds.push_back(fd);
  return fd;
}
}}}

// Independent safety barrier if SDK interposition/link closure is incorrect:
// the process cannot create any network socket or transmit any datagram.
extern "C" int socket(int, int, int) noexcept {
  ++prohibited_socket_calls;
  errno = EPERM;
  return -1;
}
extern "C" ssize_t sendto(int, const void*, size_t size, int,
                           const struct sockaddr*, socklen_t) {
  ++swallowed_sendto_calls;
  return static_cast<ssize_t>(size);
}

TEST_CASE("LivoxSdk.PartialDetectionChannelFailureReleasesResources") {
  runFailure(2, false);
}
TEST_CASE("LivoxSdk.PartialDataChannelFailureReleasesResources") {
  runFailure(4, true);
}
TEST_CASE("LivoxSdk.SuccessfulStopAllowsDetectionOnTheNextInitialization") {
  runSuccessAndRetry();
}
