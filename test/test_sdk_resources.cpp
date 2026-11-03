// Real SDK shared-library resource contracts; no SDK lifecycle is copied.
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
#include <sys/stat.h>
#include <unistd.h>
#include "livox_lidar_api.h"
#include "base/logging.h"
#define private public
#define protected public
#include "base/io_thread.h"
#include "base/multiple_io/multiple_io_epoll.h"
#include "base/network/network_util.h"
#include "base/wake_up/wake_up_pipe.h"
#include "command_handler/general_command_handler.h"
#include "data_handler/data_handler.h"
#include "logger_handler/logger_manager.h"
#include "debug_point_cloud_handler/debug_point_cloud_manager.h"
#undef protected
#undef private

namespace {
using namespace livox::lidar;
std::atomic<unsigned> network_socket_calls{0};
std::atomic<unsigned> network_sendto_calls{0};

size_t countEntries(const char* path) {
  DIR* dir = opendir(path);
  REQUIRE(dir != nullptr);
  size_t count = 0;
  while (const dirent* entry = readdir(dir))
    if (entry->d_name[0] != '.') ++count;
  closedir(dir);
  return count;
}

struct StdinHole {
  int saved;
  int original_flags;
  StdinHole() : saved(-1), original_flags(fcntl(0, F_GETFD)) {
    if (original_flags >= 0) {
      saved = fcntl(0, F_DUPFD_CLOEXEC, 64);
      REQUIRE(saved >= 0);
    } else {
      REQUIRE(errno == EBADF);
    }
    close(0);
  }
  ~StdinHole() {
    if (saved >= 0) {
      dup2(saved, 0);
      fcntl(0, F_SETFD, original_flags);
      close(saved);
    } else {
      close(0);
    }
  }
};

struct OwnedFd {
  int value;
  explicit OwnedFd(int fd = -1) : value(fd) {}
  ~OwnedFd() { if (value >= 0) close(value); }
  OwnedFd(const OwnedFd&) = delete;
  OwnedFd& operator=(const OwnedFd&) = delete;
};

struct PrivateDirectory {
  char name[64] = "/tmp/livox-sdk-resource-contract-XXXXXX";
  PrivateDirectory() { REQUIRE(mkdtemp(name) != nullptr); }
  ~PrivateDirectory() {
    for (const char* file : {".keep", "keep", "blocker", "broken.json"})
      std::remove(path(file).c_str());
    rmdir(name);
  }
  std::string path(const char* file) const { return std::string(name) + "/" + file; }
  void write(const char* file, const char* data) const {
    FILE* out = std::fopen(path(file).c_str(), "wb");
    REQUIRE(out != nullptr);
    const size_t written = std::fwrite(data, 1, std::strlen(data), out);
    const int closed = std::fclose(out);
    REQUIRE(written == std::strlen(data));
    REQUIRE(closed == 0);
  }
  std::string read(const char* file) const {
    FILE* in = std::fopen(path(file).c_str(), "rb");
    if (!in) return {};
    char buffer[64];
    const size_t length = std::fread(buffer, 1, sizeof(buffer), in);
    std::fclose(in);
    return std::string(buffer, length);
  }
};

struct CurrentDirectory {
  OwnedFd original;
  explicit CurrentDirectory(const char* directory)
      : original(open(".", O_RDONLY | O_DIRECTORY | O_CLOEXEC)) {
    REQUIRE(original.value >= 0);
    REQUIRE(chdir(directory) == 0);
  }
  ~CurrentDirectory() { fchdir(original.value); }
};

void infoChanged(uint32_t, const LivoxLidarInfo*, void*) {}
void infoText(uint32_t, uint8_t, const char*, void*) {}
void commandObserved(uint32_t, const LivoxLidarCmdPacket*, void*) {}
void pointData(uint32_t, uint8_t, LivoxLidarEthernetPacket*, void*) {}

void seedCallbacksAndPendingCommand() {
  SetLivoxLidarInfoChangeCallback(infoChanged, &network_socket_calls);
  SetLivoxLidarInfoCallback(infoText, &network_socket_calls);
  LivoxLidarAddCmdObserver(commandObserved, &network_socket_calls);
  SetLivoxLidarPointCloudCallBack(pointData, &network_socket_calls);
  SetLivoxLidarImuDataCallback(pointData, &network_socket_calls);
  LivoxLidarAddPointCloudObserver(pointData, &network_socket_calls);
  Command command;
  command.packet.cmd_type = kCommandTypeCmd;
  command.packet.seq_num = 0x13579;
  GeneralCommandHandler::GetInstance().AddCommand(command);
  REQUIRE(GeneralCommandHandler::GetInstance().commands_.count(0x13579) == 1);
}

void checkCallbacksAndCommandsCleared() {
  const auto& command = GeneralCommandHandler::GetInstance();
  const auto& data = DataHandler::GetInstance();
  CHECK(command.commands_.empty());
  CHECK(command.livox_lidar_info_change_cb_ == nullptr);
  CHECK(command.livox_lidar_info_change_client_data_ == nullptr);
  CHECK(command.livox_lidar_info_cb_ == nullptr);
  CHECK(command.livox_lidar_info_client_data_ == nullptr);
  CHECK(command.cmd_observer_cb_ == nullptr);
  CHECK(command.cmd_observer_client_data_ == nullptr);
  CHECK_FALSE(data.point_data_callbacks_);
  CHECK(data.point_client_data_ == nullptr);
  CHECK_FALSE(data.imu_data_callbacks_);
  CHECK(data.imu_client_data_ == nullptr);
  CHECK(data.observers_.empty());
}

void runEarlyFailure(const char* path) {
  DisableLivoxSdkConsoleLogger();
  seedCallbacksAndPendingCommand();
  const bool initialized = LivoxLidarSdkInit(path);
  CHECK_FALSE(initialized);
  // These checks precede explicit Uninit: a failed public Init must release
  // pending callback ownership rather than require the caller to guess stages.
  checkCallbacksAndCommandsCleared();
  LivoxLidarSdkUninit();
  LivoxLidarSdkUninit();
  checkCallbacksAndCommandsCleared();
  CHECK(network_socket_calls.load() == 0);
  CHECK(network_sendto_calls.load() == 0);
}
}

// No test below needs a network transport. Keep a process-level barrier if a
// future SDK change accidentally takes a channel-creation or send path.
extern "C" int socket(int, int, int) noexcept {
  ++network_socket_calls;
  errno = EPERM;
  return -1;
}
extern "C" ssize_t sendto(int, const void*, size_t size, int,
                           const struct sockaddr*, socklen_t) {
  ++network_sendto_calls;
  return static_cast<ssize_t>(size);
}

TEST_CASE("LivoxSdkResources.ColdAndRepeatedIoCleanupAreHarmless") {
  const size_t original_fds = countEntries("/proc/self/fd");
  const size_t original_tasks = countEntries("/proc/self/task");
  {
    IOThread cold;
    cold.Join();
    cold.Join();
    cold.Uninit();
    cold.Uninit();
  }
  {
    IOLoop cold(false, false);
    cold.Uninit();
    cold.Uninit();
  }
  {
    IOThread live;
    REQUIRE(live.Init(false, false));
    REQUIRE(live.Start());
    live.Join();
    live.Join();
    live.Uninit();
    live.Uninit();
  }
  CHECK(countEntries("/proc/self/task") == original_tasks);
  CHECK(countEntries("/proc/self/fd") == original_fds);
}

TEST_CASE("LivoxSdkResources.RepeatedPipeDestroyDoesNotCloseReusedDescriptors") {
  OwnedFd reused_read;
  OwnedFd reused_write;
  OwnedFd backing;
  {
    WakeUpPipe pipe;
    REQUIRE(pipe.PipeCreate());
    const int read_fd = pipe.pipe_out_;
    const int write_fd = pipe.pipe_in_;
    REQUIRE(pipe.PipeDestroy());
    OwnedFd source(eventfd(0, EFD_CLOEXEC));
    REQUIRE(source.value >= 0);
    backing.value = fcntl(source.value, F_DUPFD_CLOEXEC,
                         std::max(read_fd, write_fd) + 16);
    REQUIRE(backing.value >= 0);
    REQUIRE(dup2(backing.value, read_fd) == read_fd);
    REQUIRE(dup2(backing.value, write_fd) == write_fd);
    reused_read.value = read_fd;
    reused_write.value = write_fd;
    // The source may itself have reused an old pipe fd; its temporary owner
    // must no longer close that reused slot before this contract checks it.
    if (source.value == read_fd || source.value == write_fd) source.value = -1;
    REQUIRE(pipe.PipeDestroy());
    CHECK(fcntl(reused_read.value, F_GETFD) != -1);
    CHECK(fcntl(reused_write.value, F_GETFD) != -1);
  }
  CHECK(fcntl(reused_read.value, F_GETFD) != -1);
  CHECK(fcntl(reused_write.value, F_GETFD) != -1);
}

TEST_CASE("LivoxSdkResources.PipeDescriptorZeroCanWakeDrainAndClose") {
  StdinHole hole;
  {
    WakeUpPipe pipe;
    REQUIRE(pipe.PipeCreate());
    REQUIRE(pipe.GetPipeOut() == 0);
    REQUIRE(pipe.WakeUp());
    REQUIRE(pipe.Drain());
    REQUIRE(pipe.PipeDestroy());
    const int status = fcntl(0, F_GETFD);
    const int saved_errno = errno;
    CHECK(status == -1);
    CHECK(saved_errno == EBADF);
    REQUIRE(pipe.PipeDestroy());
  }
  CHECK(fcntl(0, F_GETFD) == -1);
}

TEST_CASE("LivoxSdkResources.EpollAndCloseSockOwnDescriptorZero") {
  StdinHole hole;
  {
    MultipleIOEpoll poll;
    REQUIRE(poll.PollCreate(4));
    CHECK(poll.epoll_fd_ == 0);
    poll.PollDestroy();
    CHECK(fcntl(0, F_GETFD) == -1);
    poll.PollDestroy();
  }
  const int fd = eventfd(0, EFD_CLOEXEC);
  REQUIRE(fd == 0);
  util::CloseSock(fd);
  const int status = fcntl(0, F_GETFD);
  const int saved_errno = errno;
  CHECK(status == -1);
  CHECK(saved_errno == EBADF);
}

TEST_CASE("LivoxSdkResources.ParseFailureClearsCallbacksAndPendingCommand") {
  PrivateDirectory directory;
  directory.write("broken.json", "{ broken");
  runEarlyFailure(directory.path("broken.json").c_str());
}

TEST_CASE("LivoxSdkResources.MissingConfigClearsCallbacksAndPendingCommand") {
  PrivateDirectory directory;
  runEarlyFailure(directory.path("missing.json").c_str());
}

TEST_CASE("LivoxSdkResources.FailedLoggerInitNeverRenamesUnrelatedHiddenFiles") {
  PrivateDirectory directory;
  directory.write(".keep", "private-marker\n");
  directory.write("blocker", "regular-file\n");
  directory.write("broken.json", "{ broken");
  CurrentDirectory current(directory.name);
  DisableLivoxSdkConsoleLogger();
  InitLogger();
  std::shared_ptr<LivoxLidarLoggerCfg> config(new LivoxLidarLoggerCfg());
  config->lidar_log_enable = true;
  config->lidar_log_cache_size = 1;
  config->lidar_log_path = directory.path("blocker/child");
  const bool logger_initialized = LoggerManager::GetInstance().Init(config);
  CHECK_FALSE(logger_initialized);
  CHECK(directory.read(".keep") == "private-marker\n");
  CHECK(access(directory.path("keep").c_str(), F_OK) == -1);
  const bool sdk_initialized = LivoxLidarSdkInit(directory.path("broken.json").c_str());
  CHECK_FALSE(sdk_initialized);
  CHECK(directory.read(".keep") == "private-marker\n");
  CHECK(access(directory.path("keep").c_str(), F_OK) == -1);
  CHECK_FALSE(LoggerManager::GetInstance().GetLogEnable());
  CHECK(LoggerManager::GetInstance().cycle_delete_thread_ == nullptr);
  LivoxLidarSdkUninit();
  CHECK(directory.read(".keep") == "private-marker\n");
  CHECK(access(directory.path("keep").c_str(), F_OK) == -1);
  CHECK(network_socket_calls.load() == 0);
  CHECK(network_sendto_calls.load() == 0);
}

TEST_CASE("LivoxSdkResources.FailureRollbackPreservesOtherRegisteredLoggers") {
  constexpr const char* task_name = "livox-resource-contract-external-task";
  REQUIRE(spdlog::get("console") == nullptr);
  REQUIRE(spdlog::get(task_name) == nullptr);
  REQUIRE(::logger == nullptr);
  const std::vector<spdlog::sink_ptr> no_sinks;
  const auto external_console = std::make_shared<spdlog::logger>(
      "console", no_sinks.begin(), no_sinks.end());
  const auto external_task = std::make_shared<spdlog::logger>(
      task_name, no_sinks.begin(), no_sinks.end());
  spdlog::register_logger(external_console);
  spdlog::register_logger(external_task);
  struct RegistryCleanup {
    std::shared_ptr<spdlog::logger> console;
    std::shared_ptr<spdlog::logger> task;
    ~RegistryCleanup() {
      if (spdlog::get(console->name()) == console) spdlog::drop(console->name());
      if (spdlog::get(task->name()) == task) spdlog::drop(task->name());
    }
  } cleanup{external_console, external_task};

  DisableLivoxSdkConsoleLogger();
  InitLogger();
  const auto sdk_logger = ::logger;
  CHECK(sdk_logger != nullptr);
  CHECK(sdk_logger != external_console);
  InitLogger();
  CHECK(::logger == sdk_logger);
  CHECK(spdlog::get("console") == external_console);
  CHECK(spdlog::get(task_name) == external_task);
  PrivateDirectory directory;
  runEarlyFailure(directory.path("missing.json").c_str());
  CHECK(::logger == nullptr);
  CHECK(spdlog::get("console") == external_console);
  CHECK(spdlog::get(task_name) == external_task);
}

TEST_CASE("LivoxSdkResources.FailureRollbackJoinsActiveAuxWorkersAndClosesTheirFiles") {
  PrivateDirectory directory;
  CurrentDirectory current(directory.name);
  struct GeneratedFiles {
    std::string directory;
    std::string debug_file;
    ~GeneratedFiles() {
      if (!debug_file.empty()) std::remove(debug_file.c_str());
      std::remove((directory + "/livox_log.txt").c_str());
      rmdir((directory + "/lidar_log").c_str());
    }
  } files{directory.name, {}};
  struct LogFlagRestore {
    bool save = is_save_log_file;
    bool console = is_console_log_enable;
    ~LogFlagRestore() {
      is_save_log_file = save;
      is_console_log_enable = console;
    }
  } log_flags;
  constexpr const char* task_name = "livox-resource-contract-active-external";
  REQUIRE(::logger == nullptr);
  REQUIRE(spdlog::get("console") == nullptr);
  REQUIRE(spdlog::get(task_name) == nullptr);
  const std::vector<spdlog::sink_ptr> no_sinks;
  const auto external_console = std::make_shared<spdlog::logger>(
      "console", no_sinks.begin(), no_sinks.end());
  const auto external_task = std::make_shared<spdlog::logger>(
      task_name, no_sinks.begin(), no_sinks.end());
  spdlog::register_logger(external_console);
  spdlog::register_logger(external_task);
  struct RegistryCleanup {
    std::shared_ptr<spdlog::logger> console;
    std::shared_ptr<spdlog::logger> task;
    ~RegistryCleanup() {
      if (spdlog::get(console->name()) == console) spdlog::drop(console->name());
      if (spdlog::get(task->name()) == task) spdlog::drop(task->name());
    }
  } registry{external_console, external_task};
  (void)LoggerManager::GetInstance();
  (void)DebugPointCloudManager::GetInstance();
  const size_t original_tasks = countEntries("/proc/self/task");
  const size_t original_fds = countEntries("/proc/self/fd");

  DisableLivoxSdkConsoleLogger();
  SaveLivoxLidarSdkLoggerFile();
  InitLogger();
  const bool sdk_logger_present = ::logger != nullptr;
  // Do not retain a second shared_ptr to the SDK logger: its rotating sink
  // must really close when public API rollback releases the SDK owner.
  if (::logger) ::logger->info("sdk-active-resource-contract-marker");
  struct stat sdk_log_stat{};
  const bool sdk_file_opened =
      stat(directory.path("livox_log.txt").c_str(), &sdk_log_stat) == 0 &&
      sdk_log_stat.st_size > 0;

  std::shared_ptr<LivoxLidarLoggerCfg> config(new LivoxLidarLoggerCfg());
  config->lidar_log_enable = true;
  config->lidar_log_cache_size = 1;
  config->lidar_log_path = directory.name;
  const bool logger_initialized = LoggerManager::GetInstance().Init(config);
  const bool logger_comm_prepared = LoggerManager::GetInstance().comm_port_ != nullptr;
  // Leave LoggerManager's detected-device table empty, so StopAllLogger needs
  // no network command; exercise its active deletion worker and owned CommPort.

  DetectionData detection{};
  const char serial[] = "SDK_RESOURCE_01";  // 15 bytes plus terminator fits sn[16]
  std::memcpy(detection.sn, serial, sizeof(serial));
  detection.dev_type = kLivoxLidarTypeMid360;
  detection.lidar_ip[0] = 127;
  detection.lidar_ip[3] = 81;
  detection.cmd_port = 56100;
  const uint32_t handle = 0x5100007f;
  auto& debug = DebugPointCloudManager::GetInstance();
  const bool debug_path_set = debug.SetStorePath(directory.name);
  debug.AddDevice(handle, &detection);
  const bool debug_enabled = debug.Enable(true);
  std::array<uint8_t, 37> payload{};
  for (size_t i = 0; i < payload.size(); ++i) payload[i] = static_cast<uint8_t>(i + 1);
  debug.Handler(handle, 0, payload.data(), payload.size());

  const std::string expected_prefix = "lidar_" + std::to_string(handle) + "_";
  const std::string expected_suffix = ".LivoxDebugPointCloudData";
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  bool debug_file_complete = false;
  while (!debug_file_complete && std::chrono::steady_clock::now() < deadline) {
    DIR* dir = opendir(directory.name);
    if (dir) {
      while (const dirent* entry = readdir(dir)) {
        const std::string name = entry->d_name;
        if (name.compare(0, expected_prefix.size(), expected_prefix) != 0 ||
            name.size() < expected_suffix.size() ||
            name.compare(name.size() - expected_suffix.size(), expected_suffix.size(),
                         expected_suffix) != 0) continue;
        files.debug_file = directory.path(name.c_str());
        struct stat status{};
        if (stat(files.debug_file.c_str(), &status) == 0 && S_ISREG(status.st_mode) &&
            status.st_size == static_cast<off_t>(
                sizeof(LivoxLidarDebugPointCloudFileHeader) + payload.size()))
          debug_file_complete = true;
      }
      closedir(dir);
    }
    if (!debug_file_complete) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  // File discovery/stat are the observation boundary. Never inspect the writer's
  // ordinary string/file_handle_/file_size_ members from this thread.
  const size_t active_tasks = countEntries("/proc/self/task");
  const size_t active_fds = countEntries("/proc/self/fd");
  const bool initialized = LivoxLidarSdkInit(directory.path("missing.json").c_str());
  const size_t tasks_after = countEntries("/proc/self/task");
  const size_t fds_after = countEntries("/proc/self/fd");

  CHECK(sdk_logger_present);
  CHECK(sdk_file_opened);
  CHECK(logger_initialized);
  CHECK(logger_comm_prepared);
  CHECK(debug_path_set);
  CHECK(debug_enabled);
  CHECK(debug_file_complete);
  CHECK(active_tasks == original_tasks + 2);
  CHECK(active_fds >= original_fds + 2);
  CHECK_FALSE(initialized);
  CHECK(tasks_after == original_tasks);
  CHECK(fds_after == original_fds);
  CHECK(::logger == nullptr);
  CHECK_FALSE(LoggerManager::GetInstance().GetLogEnable());
  CHECK(LoggerManager::GetInstance().cycle_delete_thread_ == nullptr);
  CHECK(LoggerManager::GetInstance().comm_port_ == nullptr);
  CHECK(LoggerManager::GetInstance().devices_info_.empty());
  CHECK(LoggerManager::GetInstance().handlers_.empty());
  CHECK_FALSE(debug.enable_.load());
  CHECK(debug.devices_info_.empty());
  CHECK(debug.handlers_.empty());
  CHECK(debug.path_.empty());
  CHECK(spdlog::get("console") == external_console);
  CHECK(spdlog::get(task_name) == external_task);
  CHECK(network_socket_calls.load() == 0);
  CHECK(network_sendto_calls.load() == 0);
}
