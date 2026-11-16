# Livox SDK2（slam_ws 运行时组件）

本目录只保留 ROS 2 驱动需要的 SDK 核心库和协议头。独立样例、固件升级演示和
上游开发文档不参与工作区构建，避免把无关程序带入 `slam_ws`。

## 构建

从工作区根目录：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to livox_ros_driver2 \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

SDK 核心由 `sdk_core/` 编译，驱动通过 `find_package(Livox-SDK2)` 使用工作区安装的
库和 `include/` 里的 C API。设备配置、ROS 话题和启动方式见
[`../livox_ros_driver2/README.md`](../livox_ros_driver2/README.md)。

## 验证
SDK 核心通过独立静态/共享库构建验证；驱动与 Keel 的 ROS 链路由工作区级测试负责。
