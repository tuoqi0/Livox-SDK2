
#ifndef LIVOX_NONCOPYABLE_H_
#define LIVOX_NONCOPYABLE_H_

namespace livox {
namespace lidar {

class noncopyable {
 protected:
  noncopyable() {}
  ~noncopyable() {}

 private:
  noncopyable(const noncopyable &);
  noncopyable &operator=(const noncopyable &);
};

} // namespace lidar
}  // namespace livox
#endif  // LIVOX_NONCOPYABLE_H_
