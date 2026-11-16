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

SDK 的生命周期、资源清理和协议包边界测试使用真实核心实现，但不连接雷达：

```bash
colcon build --base-paths src --packages-select Livox-SDK2 \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select Livox-SDK2 --event-handlers console_direct+
colcon test-result --verbose
```

`test_sdk_lifecycle`、`test_sdk_resources` 和 `test_sdk_packet_validation` 覆盖初始化失败、
重复退出、线程/句柄清理和坏包边界；`test_sdk_wire_layout.cpp` 只作为编译期协议布局
合同，不注册独立测试。它们不代替实机吞吐、时间同步或 SLAM 精度验收。
