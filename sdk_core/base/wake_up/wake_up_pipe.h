
#ifndef WAKE_UP_PIPE_H_
#define WAKE_UP_PIPE_H_

namespace livox {
namespace lidar {

class WakeUpPipe {
 public:
  WakeUpPipe(): pipe_in_(-1), pipe_out_(-1) {}
  virtual ~WakeUpPipe();
  bool PipeCreate();
  bool PipeDestroy();
  bool WakeUp();
  bool Drain();
  int GetPipeOut() { return pipe_out_; }
 protected:
  int pipe_in_;
  int pipe_out_;
};

} // namespace lidar
}   // namespace livox

#endif  // WAKE_UP_PIPE_H_