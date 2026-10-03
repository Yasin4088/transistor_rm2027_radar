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

衰减器和 SMA 同轴线到位后，首次真实解码只测信息波：

```bash
RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true \
RM_RADIO_CABLED_LOOP_CONFIRM=true \
RM_RADIO_EXTERNAL_ATTENUATION_DB=40 \
GENERATED_LINK_TEST_CASES=broadcast \
./radio/apps/lab_tx/run_generated_link_test.sh
```

此脚本默认使用 NanoSDR-B (`192.168.3.1`) 发射，PZSDR
(`192.168.1.10`) 接收，并自动统计有效帧、CRC 和 `0x0A01~0x0A05`
解析结果。没有串接外部衰减器时不得运行。

## 三机真实闭环验收

完整的台架链路是：

```text
Nano-B TX -- 50Ω SMA同轴线 -- 外部衰减器总计≥40dB -- PZSDR RX1
Nano-A RX2 独立接入系统（本项主要确认三机可同时在线）
PZSDR RX1 -> 0x0A01解码 -> RadarInfoToClient -> 坐标融合 -> 0x0305
```

参与同轴闭环的端口全部拆掉天线。衰减值可以由两个 20 dB 固定衰减器串联
得到；脚本按填写的总外部衰减量校验，板内 TX 衰减不计入这 40 dB。

默认验收会真实接收和发射，但 `0x0305` 只走裁判串口 dry-run：

```bash
RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true \
RM_RADIO_CABLED_LOOP_CONFIRM=true \
RM_RADIO_EXTERNAL_ATTENUATION_DB=40 \
./scripts/test_hardware_radio_loop.sh
```

脚本先确认三块 SDR 的 URI 均可达且互不相同，然后只启动 Nano-B 的信息波
发射，验证以下四级证据：物理解出的 `0x0A01`、`RadarInfoToClient`、融合结果
实际选择 `radio` 坐标，以及裁判节点生成 `0x0305` dry-run ACK。任何退出路径
都会先终止 TX，再把 Nano-B 设为最大硬件衰减并关闭 TX LO。

只有裁判系统已连接并确认允许写串口时，才额外执行：

```bash
RM_RADIO_LAB_TX_ANTENNA_CONFIRM=true \
RM_RADIO_CABLED_LOOP_CONFIRM=true \
RM_RADIO_EXTERNAL_ATTENUATION_DB=40 \
RM_RADIO_REAL_REFEREE_CONFIRM=true \
./scripts/test_hardware_radio_loop.sh --real-referee
```

单独启动安全门控后的 Nano-B 信息波 TX，可使用
`radio/apps/lab_tx/start_broadcast_tx.sh`；它不会启动干扰波 TX。

## 回退

集成前代码固定在 Git 分支：

```text
backup/pre-radio-integration-20260919
```

当前集成分支为：

```text
feature/integrated-radio-loop
```
