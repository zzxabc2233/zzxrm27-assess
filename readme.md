# 27赛季自瞄算法项目

这是一个面向 RoboMaster 赛季的视觉自瞄开源项目，覆盖装甲板识别、目标状态估计、轨迹规划、云台控制与开火决策等完整链路。项目强调模块化、可调试性与工程化，便于学习、二次开发与赛场部署。

---

## 项目简介

本项目实现了一个完整的视觉组自瞄系统，包含：

- 多线程相机驱动与时间戳同步
- 装甲板识别与位姿解算（支持离线视频与在线摄像头）
- 目标状态估计（如 EKF）
- 自瞄轨迹规划与开火决策
- 下位机通信协议与控制指令发送
- 标定、录制、回放与丰富的测试程序

项目默认不依赖 ROS，保留与上层/导航系统对接的接口，降低新手上手成本。

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

- Ubuntu 22.04
- C++17 / CMake
- OpenCV
- Eigen
- Ceres（可选，若使用非线性优化）
- spdlog, yaml-cpp, nlohmann-json
- libusb、相机厂商 SDK（MindVision/Hik 等）
- OpenVINO 或其他推理引擎（若使用 NN 检测）

具体依赖请参考 CMakeLists.txt 与源代码 include，根据需要安装相应 dev 包或第三方 SDK。

---

## 快速开始

1. 克隆仓库：
```bash
git clone <repo-url>
cd sp_vision_25-main
```

2. 安装常用依赖（示例）：
```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev libusb-1.0-0-dev nlohmann-json3-dev
```
根据相机与推理需求另行安装 SDK 与库（MindVision/Hik/OpenVINO/Ceres 等）。

3. 编译：
```bash
cmake -S . -B build
cmake --build build -j"$(nproc)"
```

4. 运行示例（示例名称以实际 build 产物为准）：
```bash
./build/auto_aim_test
```

注：若使用特定相机或推理库，请先安装相应 SDK 并配置 configs/ 下参数。

---

## 常用测试入口

- tests/auto_aim_test.cpp：整条自瞄链路测试  
- tests/planner_test.cpp：轨迹规划单元测试  
- tests/camera_test.cpp：相机采集测试  
- tests/handeye_test.cpp：手眼标定测试  
- tests/detector_video_test.cpp：离线视频检测测试

优先通过这些测试验证环境与硬件连接，再进行整套运行调试。

---

## 配置与标定

- 修改 configs/ 下对应 YAML/JSON 文件以适配相机、控制板与几何参数  
- 使用 calibration/ 下工具完成相机内参与手眼标定  
- 根据机电与弹道特性调整规划与开火延迟参数

---

## 注意事项

- 部分模块依赖厂商相机 SDK 或专有推理库，脱离相应 SDK 无法完整运行  
- 在真实机器人上部署前请先使用离线视频与仿真环境充分验证参数  
- 若需 ROS 集成，可在 io/ 或上层增加桥接节点，当前核心代码不依赖 ROS

---

## 贡献与问题反馈

欢迎提交 Issues 或 Pull Requests；若需快速交流，请在仓库中留下联系方式或项目维护者信息。

---

感谢使用与关注，祝开发顺利。
