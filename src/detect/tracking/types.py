from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional, Tuple

import numpy as np


Box = Tuple[float, float, float, float]
Point = Tuple[float, float]


class TrackState(str, Enum):
    TENTATIVE = "tentative"
    CONFIRMED = "confirmed"
    LOST = "lost"
    REMOVED = "removed"


class TargetState(str, Enum):
    MEASURED = "measured"
    OCCLUSION_HOLD = "occlusion_hold"
    BLIND_PREDICTION = "blind_prediction"
    MISSING = "missing"


@dataclass(frozen=True)
class CarDetection:
    box: Box
    confidence: float


@dataclass(frozen=True)
class ArmorEvidence:
    track_id: int
    label: str
    confidence: float
    box: Box
    light_color: Optional[str] = None
    light_confidence: float = 0.0
    light_kind: Optional[str] = None
    light_bars: int = 0
    color_committed: bool = False


@dataclass(frozen=True)
class TargetOutput:
    name: str
    state: TargetState
    position_cm: Optional[Point]
    display_xy: Optional[Point]
    confidence: float
    track_id: Optional[int]
    car_box: Optional[Box] = None
    armor_box: Optional[Box] = None

    @property
    def valid(self) -> bool:
        if self.position_cm is None or self.state == TargetState.MISSING:
            return False
        x, y = self.position_cm
        return (x != 0 or y != 0) and 0 <= x <= 2800 and 0 <= y <= 1500


@dataclass
class AlgorithmFrameResult:
    targets: Dict[str, TargetOutput]
    annotated_image: np.ndarray
    diagnostics: Dict[str, object] = field(default_factory=dict)
