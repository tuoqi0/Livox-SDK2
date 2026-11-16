
#ifndef LIVOX_FILE_MANAGER_
#define LIVOX_FILE_MANAGER_

#include <string>
#include <vector>
#include <map>
#include <cstdint>

namespace livox {
namespace lidar {

uint64_t GetDirTotalSize(const std::string& dir_name);

bool GetFileNames(const std::string& dir_name, std::multimap<std::string, std::string>& files_name);
bool ChangeHiddenFiles(const std::string& dir_name);
bool ChangeCurrentFileName(const std::string& dir_name, std::string file_name);
bool StoreFileName(const char* filename, std::multimap<std::string, std::string>& files_name);
bool DeleteHidFiles(const std::string& dir_name);

bool MakeDirecotory(std::string dir);

bool IsDirectoryExits(std::string dir);

} // namespace lidar
}  // namespace livox

#endif  // LIVOX_FILE_MANAGER_
