# 27赛季自瞄算法项目

面向 RoboMaster 赛季的视觉自瞄开源项目，覆盖装甲板识别、目标状态估计、轨迹规划、云台控制与开火决策等完整链路。
原项目来自同济大学战队，已适配本实验室硬件。

---

## 本实验室适配状态

| 模块 | 状态 | 说明 |
|------|------|------|
| 相机驱动 | 完成 | 大恒 MER-139-210U3C（Galaxy SDK），1280x1024 @ ~220fps |
| 内参标定 | 完成 | 重投影误差 0.27px |
| 外参标定 | 完成 | 已写入 `galaxy_standard.yaml` |
| 串口通信 | 完成 | CH340 USB 串口，115200 baud，协议与云台 Gimbal 共用 struct |
| YOLO 检测 | 待验证 | 模型加载无崩溃，需实弹测试 |
| PnP 解算 | 通过 | auto_aim_test 686 帧 0 错误 |
| 单发模式 | 已实现 | Rising edge 检测，瞄准到位打一颗 |
| 自瞄模式切换 | **卡住** | 需选手端/裁判系统触发，或电控组加调试开关 |

---

## 项目简介

本项目实现了一个完整的视觉组自瞄系统：

- 多线程相机驱动与时间戳同步（大恒 Galaxy / 海康 HikRobot / MindVision）
- 装甲板识别与位姿解算（YOLO + 传统方法 + PnP）
- 目标状态估计（EKF 球坐标跟踪）
- 弹道解算与自瞄角度计算（重力补偿 + 飞行时间迭代）
- 云台控制指令下发（串口 VisionToGimbal 协议）
- 开火决策（单发模式 rising edge）
- 标定、录制、回放与丰富的测试程序

项目不依赖 ROS，降低新手上手成本。

---

## 主要特点

- 无 ROS 依赖：默认可直接进行视觉与控制链路开发
- 完整工作流：从相机采集到控制执行，覆盖自瞄全链路
- 模块化设计：识别、估计、规划、通信、测试等模块相对独立
- 轨迹视角自瞄：将目标、射击点与云台动作视为轨迹，通过规划提高命中率
- 可调试性强：提供离线测试、录制回放、日志、标定与单元/集成测试

---

## 核心设计思路

通过“轨迹视角”将目标运动、射击位置与云台动作统一建模：

1. 把目标运动看作“目标轨迹”
2. 把发射对应位置看作“射击轨迹”
3. 规划云台轨迹使其与射击轨迹尽量重合
4. 在规划中考虑机电限制、延迟与弹道特性以提升命中率

---

## 项目结构

```text
sp_vision_25-main
├── assets
├── calibration
├── configs
├── io
├── src
├── tasks
│   ├── auto_aim
│   ├── auto_buff
│   └── omniperception
├── tests
├── tools
├── CMakeLists.txt
└── readme.md
```

目录说明：

- calibration/：相机与手眼标定工具  
- configs/：参数与硬件配置  
- io/：相机、串口、CAN 等硬件抽象层与驱动  
- tasks/：核心任务实现（自瞄、打符等）  
- tests/：模块级与链路级测试程序  
- tools/：辅助脚本、日志与数据处理工具  

---

## 环境与依赖

建议平台与主要依赖：

- **Ubuntu 24.04**（已适配）
- C++17 / CMake
- OpenCV（>= 4.x）
- Eigen3
- yaml-cpp, fmt, spdlog, nlohmann-json
- OpenVINO（YOLO 推理）
- **libserial**（串口通信，替代原 SocketCAN）
- **大恒 Galaxy SDK（GxIAPI）**（MER-139-210U3C 相机驱动）
- libusb-1.0

具体依赖请参考 `CMakeLists.txt` 与源代码 include。

---

## 快速开始

1. 编译：
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

2. 按顺序测试（每个通过再跑下一个）：

```bash
# ① 相机测试
./build/camera_test -c=configs/galaxy_standard.yaml

# ② 串口测试（看 RX/TX 是否正常）
./build/standard configs/galaxy_standard.yaml 2>&1 | grep -v debug

# ③ 整车自瞄（需云台下位机切到自瞄模式）
./build/standard configs/galaxy_standard.yaml 2>&1 | grep -v debug
```

3. 测试模式（强制控制云台，不需自瞄模式）：
   编辑 `configs/galaxy_standard.yaml`，取消注释 `test_yaw` / `test_pitch`，然后跑 standard。

---

## 硬件连接

```
[大恒相机 MER-139] ──USB3.0──┐
                              ├── 上位机（Ubuntu 24.04）
[CH340 串口模块]  ──USB──────┘   运行 standard 程序
       │
       └── TX/RX ── 云台下位机
```

- 相机：USB3.0 直连，`lsusb` 应看到 `Daheng Imaging`
- 串口：CH340 USB-TTL，设备路径 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
- 串口权限：需 `sudo chmod 666 /dev/ttyUSB0` 或将用户加入 `dialout` 组

---

## 已知问题与调试笔记

### 1. 云台下位机不切自瞄模式（当前阻塞项）

**现象**：RX 帧 mode 字节始终为 `0x00`（idle），上位机发出的控制指令（TX mode=1）被下位机忽略，云台不响应。

**原因**：自瞄模式需选手端（RoboMaster Client）通过裁判系统无线下发，或电控组在下位机固件中直接开启。

**解决方向**：
- 方案 A：安装 Windows 选手端（RMUC RoboMaster Client），联网连接裁判系统
- 方案 B：电控组修改下位机固件，加调试开关：无裁判系统连接时默认进入自瞄模式

### 2. 云台缓慢自转（下位机问题）

**现象**：云台不受控缓慢右转，上位机 TX 帧全零（无驱动指令）。

**原因**：下位机 IMU 陀螺仪零漂或 PID 参数问题，与上位机代码无关。联系电控组排查。

### 3. Galaxy 相机间歇性检测失败

**现象**：`GXUpdateDeviceList` 偶尔返回 0 设备，重试后恢复。

**修复**：已在 `io/galaxy/galaxy.cpp` 的 `openDevice()` 中添加重试逻辑（最多 3 次，500ms 间隔，第 2 次失败后执行 GXCloseLib+GXInitLib 库重置）。

### 4. 相机输出无画面

**说明**：`standard` 程序不显示画面（纯计算+控制）。用 `camera_test -d` 查看实时视频流。

### 5. CAN → 串口迁移

**改动**：`CBoard` 类从 SocketCAN 改为 libserial 串口。协议沿用云台 Gimbal 的 `GimbalToVision`（43字节 RX）和 `VisionToGimbal`（29字节 TX），帧头 `0x5A 0xA5`，帧尾 `0x7F 0xFE`。详见 `io/cboard.hpp` 和 `io/gimbal/gimbal.hpp`。
