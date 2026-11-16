
#ifndef LIVOX_COMMAND_CALLBACK_H
#define LIVOX_COMMAND_CALLBACK_H

#include <functional>
#include <memory>
#include "livox_lidar_api.h"

namespace livox {
namespace lidar {

class CommandCallback {
 public:
  virtual ~CommandCallback() {}
  virtual void operator()(livox_status status,uint32_t handle, void *data) = 0;
};

template <class T, class ResponseType>
class MemberFunctionCallback : public CommandCallback {
 public:
  typedef void (T::*MemFn)(livox_status status, uint32_t handle, ResponseType *data);

  MemberFunctionCallback(T *cls, MemFn func) : this_(cls), func_(func) {}
  void operator()(livox_status status, uint32_t handle, void *data) {
    if (this_) {
      (this_->*func_)(status, handle, static_cast<ResponseType *>(data));
    }
  }

 private:
  T *this_;
  MemFn func_;
};

template <class T>
class MemberFunctionCallback<T, uint8_t> : public CommandCallback {
 public:
  typedef void (T::*MemFn)(livox_status status, uint32_t handle, uint8_t response);

  MemberFunctionCallback(T *cls, MemFn func) : this_(cls), func_(func) {}
  void operator()(livox_status status, uint32_t handle, void *data) {
    if (this_) {
      if (data == NULL) {
        (this_->*func_)(status, handle, 0);
      } else {
        (this_->*func_)(status, handle, static_cast<uint8_t>(reinterpret_cast<uintptr_t>(data)));
      }
    }
  }

 private:
  T *this_;
  MemFn func_;
};

template <class T, class ResponseType>
std::shared_ptr<CommandCallback> MakeCommandCallback(T *cls,
                                                       typename MemberFunctionCallback<T, ResponseType>::MemFn func) {
  std::shared_ptr<CommandCallback> cb(new MemberFunctionCallback<T, ResponseType>(cls, func));
  return cb;
}

template <class T>
class FunctionStatusCallback : public CommandCallback {
 public:
  typedef void (*Fn)(livox_status status, uint32_t handle, T *data, void *client_data);

 public:
  FunctionStatusCallback(Fn func, void *client_data) : func_(func), client_data_(client_data) {}
  void operator()(livox_status status, uint32_t handle, void *data) {
    if (func_) {
      (*func_)(status, handle, static_cast<T *>(data), client_data_);
    }
  }

 private:
  Fn func_;
  void *client_data_;
};

template <>
class FunctionStatusCallback<uint8_t> : public CommandCallback {
 public:
  typedef void (*Fn)(livox_status status, uint32_t handle, uint8_t data, void *client_data);

 public:
  FunctionStatusCallback(Fn func, void *client_data) : func_(func), client_data_(client_data) {}
  void operator()(livox_status status, uint32_t handle, void *data) {
    if (func_) {
      if (data == NULL) {
        (*func_)(status, handle, 0, client_data_);
      } else {
        (*func_)(status, handle, *(uint8_t *)data, client_data_);
      }
    }
  }

 private:
  Fn func_;
  void *client_data_;
};

template <class T>
std::shared_ptr<CommandCallback> MakeCommandCallback(typename FunctionStatusCallback<T>::Fn func, void *client_data) {
  std::shared_ptr<CommandCallback> cb(new FunctionStatusCallback<T>(func, client_data));
  return cb;
}

template <class T>
class MessageCallback : public CommandCallback {
 public:
  typedef void (*Fn)(livox_status status, uint32_t handle, T *data);

 public:
  MessageCallback(const Fn &func) : func_(func) {}
  void operator()(livox_status status, uint32_t handle, void *data) {
    if (func_) {
      (*func_)(status, handle, static_cast<T *>(data));
    }
  }

 private:
  Fn func_;
};

template <class ResponseType>
class FunctionMessageCallback : public CommandCallback {
 public:
  typedef std::function<void(livox_status, uint8_t, ResponseType *)> Fn;

 public:
  FunctionMessageCallback(const Fn &func) : func_(func) {}
  void operator()(livox_status status, uint32_t handle, void *data) {
    if (func_) {
      func_(status, handle, static_cast<ResponseType *>(data));
    }
  }

 private:
  Fn func_;
};

template <class T>
std::shared_ptr<CommandCallback> MakeMessageCallback(typename MessageCallback<T>::Fn func) {
  std::shared_ptr<CommandCallback> cb(new MessageCallback<T>(func));
  return cb;
}

template <class T>
std::shared_ptr<CommandCallback> MakeMessageCallback(typename FunctionMessageCallback<T>::Fn func) {
  std::shared_ptr<CommandCallback> cb(new FunctionMessageCallback<T>(func));
  return cb;
}

template <class T, class ResponseType>
class MemberMessageCallback : public CommandCallback {
 public:
  typedef void (T::*MemFn)(livox_status status, uint32_t handle, ResponseType *data);

  MemberMessageCallback(T *cls, MemFn func) : this_(cls), func_(func) {}
  void operator()(livox_status status,uint32_t handle, void *data) {
    if (this_) {
      (this_->*func_)(status, handle, (ResponseType *)data);
    }
  }

 private:
  T *this_;
  MemFn func_;
};

template <class T, class ResponseType>
std::shared_ptr<CommandCallback> MakeMemberMessageCallback(
    T *_this,
    typename MemberMessageCallback<T, ResponseType>::MemFn func) {
  std::shared_ptr<CommandCallback> cb(new MemberMessageCallback<T, ResponseType>(_this, func));
  return cb;
}

template <class ResponseType>
class FunctionCallback : public CommandCallback {
 public:
  typedef std::function<void(uint8_t, uint8_t, ResponseType *)> Fn;
  FunctionCallback(const Fn &func) : func_(func) {}
  void operator()(livox_status status,uint32_t handle, void *data) {
    if (func_) {
      if (data == NULL) {
        func_(status, handle, NULL);
      } else {
        func_(status, handle, (ResponseType *)data);
      }
    }
  }

 private:
  Fn func_;
};

template <>
class FunctionCallback<uint8_t> : public CommandCallback {
 public:
  typedef std::function<void(uint8_t, uint8_t, uint8_t)> Fn;
  FunctionCallback(const Fn &func) : func_(func) {}
  void operator()(livox_status status,uint32_t handle, void *data) {
    if (func_) {
      if (data == NULL) {
        func_(status, handle, 0);
      } else {
        func_(status, handle, static_cast<uint8_t>(reinterpret_cast<uintptr_t>(data)));
      }
    }
  }

 private:
  Fn func_;
};

template <class ResponseType>
std::shared_ptr<CommandCallback> MakeCommandCallback(const typename FunctionCallback<ResponseType>::Fn &func) {
  std::shared_ptr<CommandCallback> cb(new FunctionCallback<ResponseType>(func));
  return cb;
}

} // namespace lidar
}  // namespace livox
#endif  // LIVOX_COMMAND_CALLBACK_H
