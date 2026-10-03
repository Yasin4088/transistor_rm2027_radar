# 北航 Transistor 战队 RM2027 雷达站

面向 RoboMaster 2027 的视觉与无线电一体化雷达站。本仓库在
JNU-SHARK 视觉框架基础上继续开发，现已将 3D 射线定位、目标保持、盲区预测、
信息波接收、坐标融合和裁判系统上报整合到同一套工程中。

当前开发分支：`feature/raycast-3d`。

## 当前状态

| 模块 | 状态 | 说明 |
|---|---|---|
| 车辆与装甲板识别 | 已接入 | 车辆检测、ROI 批量装甲板识别、颜色门与类别门确认 |
| 3D 射线定位 | 已接入 | 像素反投影后与场地 mesh 求交，保留 2D 仿射回退模式 |
| 跟踪与盲区处理 | 已接入 | 临时阵亡保持、遮挡保持、卡尔曼滤波、速度方向盲区选点 |
| 无线电协议 | 已接入 | GFSK 解调与 `0x0A01~0x0A06` 解析 |
| 多源坐标融合 | 已接入 | 信息波坐标优先，视觉坐标回退 |
| 裁判系统输出 | 已接入 | `0x0305` 地图坐标及雷达策略指令 |
| 纯软件闭环 | 已验证 | 视觉 → UDP → ROS2 → 融合 → 裁判 dry-run |
| 三机真实射频闭环 | 待台架复测 | 验收脚本已完成，等待 SDR 重连及 ≥40 dB 外部衰减链路 |

现有相机内外参及 2026 场地 mesh 用于开发验证。更换相机、安装位置或正式
2027 场地后，必须重新标定，不能直接将仓库内参数用于比赛坐标上报。

## 系统架构

```text
海康相机
  -> 车辆检测 -> 装甲板/颜色/身份确认 -> 像素坐标
  -> 相机内参去畸变 -> 3D 射线与场地 mesh 求交 -> 视觉场地坐标
                                                        |
PZSDR RX1 -> 信息波 GFSK -> 0x0A01~0x0A05 ------------+-> 坐标融合
NanoSDR-A RX2 -> 干扰波 GFSK -> 0x0A06 ----------------+     |
                                                              v
                                                   0x0305 / 策略指令
                                                              |
                                                         裁判系统
```

视觉程序运行在 Python 3.12 环境；无线电后端使用系统 Python 3.10、ROS2
Humble 和 GNU Radio。两侧通过仅监听 `127.0.0.1` 的 UDP sidecar 交换 JSON，
避免 Python ABI 和 `rclpy` 版本冲突。

## 硬件分工

| 设备 | 默认 URI | 用途 |
|---|---|---|
| 海康工业相机 | 由 MVS SDK 枚举 | 视觉检测与定位 |
| PZSDR P201Mini / AD9361 | `ip:192.168.1.10` | RX1，接收信息波 |
| NanoSDR-A / AD9363 | `ip:192.168.2.1` | RX2，接收干扰波 |
| NanoSDR-B / AD9363 | `ip:192.168.3.1` | 仅用于实验室 LabTX |
| 裁判系统串口 | `/dev/transistor-referee` | 坐标与策略数据上报 |

比赛接收启动器不会启动任何 LabTX。NanoSDR-B 的发射功能与比赛程序分离，且有
独立的射频安全门禁。

## 目录结构

```text
transistor_rm2027_radar/
├── main.py                         # 视觉主循环
├── config/config.yaml              # 视觉、投影、融合与 UI 配置
├── raycast.py                      # 像素到 3D 场地坐标
├── solvepnp.py                     # PnP 外参标定
├── calibrate_intrinsics.py         # 相机内参标定
├── field/                          # 场地 mesh
├── models/                         # 车辆与装甲板模型
├── src/                            # 视觉侧模块
├── radio/                          # ROS2/GNU Radio 无线电后端
│   ├── apps/match_rx/              # 比赛双路接收
│   ├── apps/lab_tx/                # 隔离的实验室发射工具
│   └── src/rm_radio_ros/           # ROS2 包与协议实现
├── scripts/start_integrated_radar.sh
├── scripts/test_integrated_radio_loop.sh
└── scripts/test_hardware_radio_loop.sh
```

无线电子系统的详细配置与台架操作见 [`radio/README.md`](radio/README.md)。

## 环境要求

视觉侧推荐环境：

- Linux + NVIDIA GPU；
- Python 3.12；
- 与本机驱动匹配的 CUDA、PyTorch 和 TensorRT；
- 海康 MVS SDK（实机相机模式需要）；
- OpenCV、Open3D、PyQt5 等依赖。

无线电侧要求：

- Ubuntu 22.04 / ROS2 Humble；
- 系统 Python 3.10；
- GNU Radio 3.10、gr-iio、libiio；
- `colcon`；
- 可访问三块 SDR 的有线网络配置。

不要在 ROS2 系统 Python 与视觉 venv 之间混装依赖。启动器优先使用仓库内
`.venv/bin/python`；也可显式指定已经配置好的视觉解释器：

```bash
VISION_PYTHON=/path/to/vision-venv/bin/python \
  ./scripts/start_integrated_radar.sh --dry-run --no-panel
```

`requirements.txt` 记录当前视觉环境版本，但其中 CUDA/PyTorch/TensorRT 版本与
显卡驱动相关。在已有可用 GPU 环境中，不建议直接整包覆盖安装。

## 构建无线电后端

```bash
cd transistor_rm2027_radar
./radio/build.sh
```

该脚本使用系统 Python 和 ROS2 Humble，不会激活视觉 venv。构建产物位于
`radio/build/`、`radio/install/` 和 `radio/log/`，均不作为源代码提交。

## 快速验证

### 纯软件闭环

以下测试不会访问 SDR，也不会写真实裁判串口：

```bash
./scripts/test_integrated_radio_loop.sh
```

它会自动检查：

1. 视觉遥测能否通过 UDP 进入 ROS2；
2. 融合节点是否在线；
3. 雷达策略请求能否收到 dry-run ACK；
4. 是否生成合法的 `0x0305` 帧及 dry-run ACK。

### 启动完整 dry-run

```bash
./scripts/start_integrated_radar.sh --dry-run --no-panel --no-browser
```

默认读取 `config/config.yaml` 中的测试视频；测试视频不可用时可回退到静态图片。

## 实机运行

先在 [`config/config.yaml`](config/config.yaml) 中确认：

```yaml
global:
  state: 'R'          # 或 'B'
  camera_mode: 'hik'

projection:
  mode: 'raycast'     # 也可切换为 affine 回退

referee:
  transport: 'radio_ros'
  bridge_backend: 'udp'
```

然后启动：

```bash
./scripts/start_integrated_radar.sh
```

常用参数：

- `--no-panel`：不启动无线电 Web 面板；
- `--no-browser`：不自动打开浏览器；
- `--headless`：关闭 GNU Radio 原生图形窗口；
- `--radio-only`：只启动无线电后端，不启动视觉进程；
- `--dry-run`：禁止真实 SDR 和裁判串口 I/O。

## 三机台架验收与射频安全

真实闭环只能使用 50 Ω 同轴连接：

```text
NanoSDR-B TX -> SMA 同轴线 -> 外部固定衰减器总计 ≥40 dB -> PZSDR RX1
```

两个 20 dB、50 Ω、覆盖 433 MHz 且功率额定值足够的固定衰减器可串联为
40 dB。参与同轴闭环的端口必须拆掉天线；板内软件衰减不计入外部 40 dB
安全门槛。

默认验收会真实收发射频，但裁判输出保持 dry-run：

```bash
RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true \
RM_RADIO_CABLED_LOOP_CONFIRM=true \
RM_RADIO_EXTERNAL_ATTENUATION_DB=40 \
./scripts/test_hardware_radio_loop.sh
```

脚本会在发射前检查三块 SDR 均可达且 URI 互不重复，并验证：

```text
物理 0x0A01 解码
  -> RadarInfoToClient
  -> 融合结果实际选择 radio 坐标
  -> 生成 0x0305 裁判帧 ACK
```

退出时会先停止 TX，再将 NanoSDR-B 设置为最大硬件衰减并关闭 TX LO。
缺少任一确认、外部衰减低于 40 dB 或设备不可达时，脚本都会拒绝发射。

仅在裁判系统已连接、串口权限正确且确认允许写入时，才可进一步使用：

```bash
RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true \
RM_RADIO_CABLED_LOOP_CONFIRM=true \
RM_RADIO_EXTERNAL_ATTENUATION_DB=40 \
RM_RADIO_REAL_REFEREE_CONFIRM=true \
./scripts/test_hardware_radio_loop.sh --real-referee
```

## 标定与坐标系

3D 定位链路为：

```text
图像像素 -> 相机内参/畸变校正 -> 相机射线 -> 外参 R/T
         -> 场地 mesh 求交 -> 内部地图坐标 -> 裁判坐标
```

- `camera_intrinsics.npz`：相机内参；
- `extrinsics.npz`：相机相对场地的外参；
- `field/*.PLY`：用于射线求交的场地 mesh；
- `homography_matrix/*.npy`：2D 仿射回退标定。

相机、更换镜头、分辨率、架设位置或场地模型发生变化时，应重新执行内参/外参
标定。射线未命中 mesh 时系统返回无效点，不会把图像像素误当作场地坐标上报。

## 测试

视觉单元测试：

```bash
PYTHONPATH=src /path/to/vision-venv/bin/python \
  -m unittest discover -s tests -v
```

无线电单元测试：

```bash
set +u
source /opt/ros/humble/setup.bash
source radio/install/setup.bash
set -u
PYTHONPATH=radio/src/rm_radio_ros:radio:${PYTHONPATH:-} \
  python3 -m pytest -q radio/src/rm_radio_ros/test
```

当前基线结果：视觉 83 项、无线电 283 项全部通过。

## 回退

无线电整合前的稳定提交为：

```text
3b271a1
```

本地还保留 `backup/pre-radio-integration-20260919` 分支。需要对照旧版时可新建
工作分支指向该提交，不必改写 `feature/raycast-3d` 历史。

## 开源来源与许可证

- 视觉基础框架参考 JNU-SHARK 雷达开源方案；
- 3D 射线定位思路参考 HKUST RoboMaster 雷达开源实现；
- `radio/` 的 GNU Radio/ROS2 基础代码引入自 JNU-SHARK，准确上游提交及改造
  说明见 [`radio/UPSTREAM.md`](radio/UPSTREAM.md)；
- 无线电上游 MIT 许可证保存在 [`radio/LICENSE.upstream`](radio/LICENSE.upstream)。

仓库主体许可证见 [`LICENSE`](LICENSE)。
