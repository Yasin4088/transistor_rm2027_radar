# 北航 Transistor 战队 RM2027 雷达站

本仓库现同时包含视觉雷达和无线电后端，源码不再导入工作区中的
其他参考仓库。视觉基础框架借鉴了 JNU-SHARK 的开源工作；无线电源码的
准确上游版本和 MIT 归属见 [`radio/UPSTREAM.md`](radio/UPSTREAM.md)。

## 一体化链路

```text
海康相机 -> 车辆/装甲板检测 -> 3D 定位 -> 视觉遥测
                                              |
PZSDR/NanoSDR -> GFSK -> 0x0A01~0x0A06 ------+-> 坐标融合
                                                   -> 0x0305/策略指令
                                                   -> 裁判系统
```

无线电后端位于 [`radio/`](radio/README.md)。视觉 Python 3.12 与 ROS2 Humble
Python 3.10 通过 localhost UDP sidecar 隔离，解决 `rclpy` ABI 不兼容问题。

## 启动

首次构建无线电 ROS2 包：

```bash
./radio/build.sh
```

纯软件闭环（不访问 SDR/真实串口）：

```bash
./scripts/start_integrated_radar.sh --dry-run --no-panel
```

实机接收：

```bash
./scripts/start_integrated_radar.sh
```

LabTX 必须单独启动，不会随比赛接收链路启动。

启动器优先使用本仓库 `.venv/bin/python`。当前机器为避免重复安装
TensorRT/CUDA，可以显式传入已有视觉环境：

```bash
VISION_PYTHON=/path/to/python ./scripts/start_integrated_radar.sh
```

## 分支回退

- 集成分支：`feature/integrated-radio-loop`
- 集成前回退点：`backup/pre-radio-integration-20260919`
