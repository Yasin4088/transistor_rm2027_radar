"""多源坐标融合（精简自 TCR position_fusion.py）

输入：
  - 信息波坐标（0x0A01，UWB 定位结果，裁判坐标 cm）
  - 视觉坐标（3D 射线定位，内部坐标 → 已转裁判坐标 cm）
  - 盲区预测点（视觉丢失时）

仲裁优先级（按新鲜度）：
  ① INFO_CURRENT 信息波新鲜（≤0.2s）→ 用（无遮挡，首选）
  ② GROUND       视觉新鲜（≤0.5s）  → 用（回退）
  ③ INFO_STALE   信息波旧（≤10s）    → 用旧的（兜底）
  ④ BLIND_GUESS  盲区预测点          → 用（最低）
  ⑤ INVALID      全无                → 不上报

坐标约定：全部为裁判坐标 (x: 0-2800, y: 0-1500) cm
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import threading
import time
from typing import Mapping, Optional, Tuple


class PositionSource(str, Enum):
    INFO_CURRENT = "info_current"
    GROUND = "ground"
    INFO_STALE = "info_stale"
    BLIND_GUESS = "blind_guess"
    INVALID = "invalid"


@dataclass(frozen=True)
class CoordinateCm:
    x: int
    y: int


@dataclass(frozen=True)
class InfoWaveSample:
    coordinate: CoordinateCm
    received_at: float
    revision: int


@dataclass(frozen=True)
class VisionSample:
    ground: Optional[CoordinateCm]      # 视觉地面坐标
    blind_guess: Optional[CoordinateCm]  # 盲区预测点
    updated_at: float


@dataclass(frozen=True)
class ResolvedPosition:
    coordinate: Optional[CoordinateCm]
    source: PositionSource
    info_age_s: Optional[float] = None


class PositionFusionBuffer:
    """多源坐标仲裁缓冲（线程安全）"""

    FIELD_MAX_X_CM = 2800
    FIELD_MAX_Y_CM = 1500

    def __init__(
        self,
        *,
        info_fresh_timeout_s: float = 0.2,
        info_stale_timeout_s: float = 10.0,
        vision_watchdog_s: float = 0.5,
        clock=time.monotonic,
    ):
        self.info_fresh_timeout_s = info_fresh_timeout_s
        self.info_stale_timeout_s = info_stale_timeout_s
        self.vision_watchdog_s = vision_watchdog_s
        self._clock = clock

        self._lock = threading.RLock()
        self._info_revision = 0
        self._info_by_name: dict[str, InfoWaveSample] = {}
        self._vision_by_name: dict[str, VisionSample] = {}
        self._last_output_info_revision: dict[str, int] = {}

    # ---------------- 数据写入 ----------------

    def update_info_wave(self, positions: Mapping[str, Tuple[int, int]],
                         received_at: Optional[float] = None) -> int:
        """信息波 0x0A01 坐标写入（positions: {机器人名: (x_cm, y_cm)}）"""
        with self._lock:
            now = self._clock() if received_at is None else received_at
            self._info_revision += 1
            accepted = 0
            for name, (x, y) in positions.items():
                coord = self._to_cm(x, y)
                if coord is None:
                    continue
                self._info_by_name[name] = InfoWaveSample(
                    coordinate=coord, received_at=now, revision=self._info_revision)
                accepted += 1
            return accepted

    def update_vision_tracks(self, vision: Mapping[str, Tuple[Optional[Tuple[int, int]], Optional[Tuple[int, int]]]],
                             updated_at: Optional[float] = None) -> int:
        """视觉坐标写入（vision: {机器人名: (地面坐标或None, 盲区点或None)}）"""
        with self._lock:
            now = self._clock() if updated_at is None else updated_at
            for name, (ground, blind) in vision.items():
                g = self._to_cm(*ground) if ground else None
                b = self._to_cm(*blind) if blind else None
                self._vision_by_name[name] = VisionSample(
                    ground=g, blind_guess=b, updated_at=now)
            return len(vision)

    def clear_info_wave(self) -> None:
        """阵营切换时清空信息波旧数据（防止换边后坐标误用）"""
        with self._lock:
            self._info_by_name.clear()
            self._last_output_info_revision.clear()

    # ---------------- 仲裁 ----------------

    def resolve(self, names: list[str]) -> dict[str, ResolvedPosition]:
        """对每辆车仲裁，返回最优坐标（按新鲜度优先级）"""
        with self._lock:
            now = self._clock()
            result = {}
            for name in names:
                info = self._info_by_name.get(name)
                vision = self._vision_by_name.get(name)

                info_age = now - info.received_at if info else None

                # ① 信息波新鲜（首选）
                if info is not None and info_age is not None and info_age <= self.info_fresh_timeout_s:
                    result[name] = ResolvedPosition(
                        info.coordinate, PositionSource.INFO_CURRENT, info_age)
                    continue

                # ② 视觉新鲜（回退）
                vision_fresh = (vision is not None and
                                now - vision.updated_at <= self.vision_watchdog_s)
                if vision_fresh and vision is not None and vision.ground is not None:
                    result[name] = ResolvedPosition(
                        vision.ground, PositionSource.GROUND)
                    continue

                # ③ 信息波旧数据（兜底）
                if info is not None and info_age is not None and info_age <= self.info_stale_timeout_s:
                    result[name] = ResolvedPosition(
                        info.coordinate, PositionSource.INFO_STALE, info_age)
                    continue

                # ④ 盲区预测（最低）
                if vision is not None and vision.blind_guess is not None:
                    result[name] = ResolvedPosition(
                        vision.blind_guess, PositionSource.BLIND_GUESS)
                    continue

                # ⑤ 无效
                result[name] = ResolvedPosition(None, PositionSource.INVALID)

            return result

    # ---------------- 工具 ----------------

    @classmethod
    def _to_cm(cls, x, y) -> Optional[CoordinateCm]:
        """坐标转 cm 并校验在场地内（0-2800/0-1500）"""
        try:
            x_cm = int(x)
            y_cm = int(y)
        except (TypeError, ValueError):
            return None
        if not (0 <= x_cm <= cls.FIELD_MAX_X_CM and 0 <= y_cm <= cls.FIELD_MAX_Y_CM):
            return None
        return CoordinateCm(x_cm, y_cm)
