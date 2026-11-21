# Livox SDK2（slam_ws 运行时组件）

本目录只保留 ROS 2 驱动需要的 SDK 核心库和协议头。独立样例、固件升级演示和
上游开发文档不参与工作区构建，避免把无关程序带入 `slam_ws`。

## 构建

从工作区根目录：

```bash
rc install -j2 --prefix install/rc livox_sdk2_package
```

SDK 核心在 `sdk_core/`。RC 安装静态库、共享库和 `include/` 里的 C API。外部 CMake
消费者通过 `find_package(Livox-SDK2 CONFIG REQUIRED)` 查找已安装的 SDK。设备配置、
ROS 话题和启动方式见
[`../livox_ros_driver2/README.md`](../livox_ros_driver2/README.md)。

## 验证
SDK 核心通过独立静态/共享库构建验证；驱动与 Keel 的 ROS 链路由工作区级测试负责。

独立 Windows SDK 继续使用随包的 CMake 入口和 Visual Studio。其源文件选择使用
Windows 网络和事件唤醒后端。Jazzy 工作区从 `rc.yaml` 构建 SDK。
