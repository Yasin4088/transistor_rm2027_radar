# Transistor RM2027 无线电后端

本目录是 2027 雷达项目的内置无线电子系统，不再依赖工作区中的参考仓库。
它包含 AD9361/AD9363 GNU Radio 流图、`0x0A01~0x0A06` 协议解析、
视觉/信息波坐标融合、裁判串口发送、LabTX 和运维面板。

## 运行时边界

- `main.py`：视觉 venv / Python 3.12。
- `radio/`：系统 Python 3.10 + ROS2 Humble + GNU Radio。
- `rm_vision_udp_bridge`：只绑定 `127.0.0.1`，在两个 Python ABI 之间转发 JSON。

数据链：

```text
PZSDR RX1 / Nano-A RX2
  -> GNU Radio GFSK
  -> 0x0A01~0x0A06 解析
  -> ROS2 radar_integration
             ^
             | localhost UDP
             |
       2027 视觉坐标
  -> 0x0305 / 0x0121 / 0x0234
  -> 裁判系统串口
```

## 构建

```bash
cd /home/elysia/robomaster/transistor_rm2027_radar
./radio/build.sh
```

## 纯软件闭环

不访问 SDR，不写真实裁判串口：

```bash
./scripts/start_integrated_radar.sh --dry-run --no-panel
```

可重复的自动闭环验证（启动 dry-run 后端，断言 UDP/ROS2/融合/
`0x0305`/策略 ACK 全部通过）：

```bash
./scripts/test_integrated_radio_loop.sh
```

## 比赛接收

```bash
./scripts/start_integrated_radar.sh
```

当前三机分工：

| 设备 | URI | 角色 |
|---|---|---|
| PZSDR P201Mini | `ip:192.168.1.10` | RX1 信息波 |
| NanoSDR-A | `ip:192.168.2.1` | RX2 干扰波 |
| NanoSDR-B | `ip:192.168.3.1` | 实验室单板 LabTX |

LabTX 不会被比赛启动器启动。发射前必须确认合法频率、假负载/衰减器、
同轴连接和功率设置，并显式设置
`RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true`。

## 回退

集成前代码固定在 Git 分支：

```text
backup/pre-radio-integration-20260919
```

当前集成分支为：

```text
feature/integrated-radio-loop
```
