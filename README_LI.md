# RM2027 雷达站 — 3D 射线定位开发记录（feature/raycast-3d 分支）

> 本文件提取自 AGENTS.md，供队友快速了解当前分支的开发内容和已知问题。

## 分支概况

`feature/raycast-3d` 在 shark（江南大学 2026）基础上做两件升级：
1. **3D 射线定位**（替代 2D 双层仿射，移植 HKUST 2025）
2. **盲区预测打分选点**（速度方向+距离，移植 HKUST guess_pts）

## 已完成

| 模块 | 说明 |
|---|---|
| `raycast.py` | `PixelToWorld`：像素→畸变校正→反投影→与场地 mesh 求交→世界坐标 |
| `solvepnp.py` | 6 点 PnP 外参标定工具（M==N 直接 ITERATIVE，M<N 暴力枚举） |
| `calibrate_intrinsics.py` + `camera_intrinsics.npz` | 棋盘格内参标定（fx=3244.05, cx=2022.90, RMS 4.09px） |
| `build_2026_mesh.py` + `field/RMUC2026_simple.PLY` | 手建 2026 粗糙场地 mesh（占位，等 2027 官方模型替换） |
| 投影层可插拔 | config `projection.mode`：`affine`（2D 仿射）/ `raycast`（3D 射线）一键切换 |
| 盲区预测升级 | `send_point_guess`：有速度→打分选点（cos_factor=0.003, d_factor=0.01）；无速度→原双点轮换 |

## 用法

```bash
# 运行（用 shark 的 venv）
/home/elysia/robomaster/shark-radar-system/shark-radar-vision/.venv/bin/python main.py

# 切投影模式（config/config.yaml）
projection.mode: 'affine'   # 2D 双层仿射（shark 原版）
projection.mode: 'raycast'  # 3D 射线定位
```

## 已修复的 bug（重要教训）

1. **盲区预测打分静默失效**：`robot_position_history` 的初始化+填充都被 `double_vulnerability_enabled` 开关守卫，开关关闭时数据为空 → 打分永不生效且无提示。
   → 修复：初始化（globals 检查）和填充都独立于开关。**教训：数据链（初始化→填充→读取）必须整体与开关解耦。**
2. **3D 射线未命中静默降级**：射线未命中 mesh 时返回"图像像素坐标"，下游当作"场地坐标"上报 → 全场坐标全错且无报错。
   → 修复：返回 None + 计数日志（每 100 次一条），3 个调用点跳过 None。
3. **假外参静默**：未标定时默认 `R=I, T=[0,0,10]`，坐标全错无提示。
   → 修复：启动时检测并打印 ⚠️ 警告。
4. **融合视觉喂入坐标系错乱**：滤波器/盲区点是内部坐标（短边,长边），融合模块校验是裁判坐标（0-2800/0-1500）→ 直接喂会被越界误杀。
   → 修复：喂入前 `_to_referee` 转换（内部→裁判）。
5. **场地尺寸硬编码**：3D 射线投影里 14.0/7.5/28/15 写死 → 换 2027 场地必须改代码。
   → 修复：改从 config `map_size` 读取。

## 多源坐标融合（信息波 UWB + 视觉）

```
src/fusion/position_fusion.py（新）：
  仲裁优先级：① 信息波 UWB(≤0.2s) → ② 视觉 3D(≤0.5s) → ③ 信息波旧(≤10s) → ④ 盲区 → ⑤ 无效
  + 坐标校验(0-2800/0-1500) + 阵营切换清空

main.py 接入（fusion.enabled 开关，默认 false）：
  feed_info_wave_positions()  ← 信息波坐标入口（待 sp_rf 对接）
  feed_vision_positions()     ← 视觉喂入（已接）
  update_send_map_entry       ← 仲裁优先（已接）

已验证：sp_rf debug 模式 0x0A06 密钥解出、0x0A01 坐标解析成功（纯软件协议正确）
待办：sp_rf RX → feed_info_wave_positions 进程对接（等 SDR/部署）
```

## 2026 实验标定结论

- 用 TCR 开源资产验证 3D 定位链路：mesh + 13 点标定 → PnP 残差 44px → 点位分布合理（链路通）
- 相机高度 5.6 vs 实际 2.5m：根因是视频相机≠标定相机（内参不匹配），非代码问题
- **精度等 2027 实机重标**（棋盘格内参 + 6 点 PnP 外参，工具已就绪）

## 已知待办

1. **外参 R/T**：到场地后 6 点 PnP 标定（solvepnp.py 已就绪）
2. **精确 mesh**：等 2027 手册/stp 发布后建（当前 2026 粗糙版仅占位；1.25GB stp 全量网格化会 OOM，需筛选主体）
3. **A/B 对比**：仿射 vs 射线（shark 自带 compare_filter 影子对比）
4. **2D 仿射也需重标**：shark 的 arrays_test_*.npy 是他队场地标定，换场地必须重标
5. **盲区预测预选点不足**：每角色仅 2 个点，需扩到 4-5 个（吊射点/银矿/对方基地/消失点附近），等 2027 场地确定
6. **盲区预测调试日志**：加 `blind_zone.debug` 开关，运行中区分"打分/轮换"模式
7. **融合对接**：sp_rf RX → `feed_info_wave_positions`（等 SDR 硬件/部署）

## 参考

- HKUST 2025：`RM2025-Radar-Algorithm`（ray_renderer.py / solvepnp.py / guess_pts.py）
- shark 2026：`shark-radar-system`（基础链路）
- TCR 2026：`TCR-RM2026-Radar-OpenSource`（多源融合 position_fusion.py，无线电为主）
- sp_rf 2026：`sp_rf_2026-OpenSource`（雷达无线链路仿真，NanoSDR 工程）
