# 无线电源码来源

`radio/` 中的 GNU Radio/ROS2 基础实现引入自 JNU-SHARK 的
`shark-radar-radio`，上游提交为：

```text
683554763a2ac13a54cc9e1c9cb336246a9ae302
```

上游按 MIT 许可证发布，许可证原文保存在
`LICENSE.upstream`。本目录已针对 Transistor RM2027 作了以下改造：

- PZSDR + 两块 NanoSDR 的三机地址和角色配置；
- `transistor.radar.*` 视觉遥测/命令协议兼容；
- Python 3.12 视觉进程与 Python 3.10 ROS2 Humble 的 localhost UDP 桥接；
- 一体化启动、自检与回退说明。
