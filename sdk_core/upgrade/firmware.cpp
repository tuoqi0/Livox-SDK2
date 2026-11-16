
#include <string.h>

#include "firmware.h"

namespace livox {
namespace lidar {

Firmware::Firmware() : data_(nullptr), file_size_(0) {
  memset((void *)&header_, 0, sizeof(header_));
  memset((void *)&tail_, 0, sizeof(tail_));
}

Firmware::~Firmware() {
  if (data_) {
    delete[] data_;
  }
}
void Firmware::Close() {
  file_.close();
}

bool Firmware::Open(const char *firmware_path) {
  if (!firmware_path) {
    return false;
  }

  file_.open(firmware_path,
             std::ios::in | std::ifstream::binary | std::ios_base::ate);
  if (!file_.is_open()) {
    printf("Open %s firmware file fail!\n", firmware_path);
    return false;
  }

  file_size_ = file_.tellg();
  file_.seekg(0, std::ios::beg);
  if (file_size_ < MiniFileSize()) {
    printf("Firmware file size is too small!\n");
    return false;
  }

  if (!ReadAndCheckHeader()) {
    printf("Firmware file header is wrong!\n");
    return false;
  }

  //printf("Firmware file total size %lu\n", file_size_);
  return true;
}

uint64_t Firmware::MiniFileSize() {
  return sizeof(header_) + sizeof(tail_) + 1;
}

bool Firmware::ReadAndCheckHeader() {
  file_.seekg(0, std::ios::beg);
  file_.read((char *)(&header_), sizeof(header_));
  printf("This firmware is used for device[%d].\n", header_.device_type);

  uint16_t crc = crc16_.mcrf4xx(
      (uint8_t *)(&header_), (uint16_t)(sizeof(header_) - sizeof(uint16_t)));
  if (crc != header_.header_checksum) {
    printf("Header checksum[%4x %4x] error!\n", crc, header_.header_checksum);
    return false;
  }

  printf("Firmware raw data size : %d\n", header_.firmware_length);
  data_ = new uint8_t[header_.firmware_length];
  file_.read((char *)(data_), header_.firmware_length);
  file_.read((char *)(&tail_), sizeof(tail_));

  if (file_) {
    printf("All firmware data have be read successfully.\n");
  } else {
    printf("Read firmware fail[%ld]!\n", file_.gcount());
  }

  return true;
}

}
}
