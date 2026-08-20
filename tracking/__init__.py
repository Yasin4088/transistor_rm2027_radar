"""Track-centric radar algorithm components.

The cascade matching design is adapted from HKUST ENTERPRIZE's MIT-licensed
RM2025 Radar Algorithm (commit f12fe91), with SHARK-specific detection,
identity, timing, and referee-output semantics.
"""

from .pipeline import TrackedRadarPipeline
from .types import AlgorithmFrameResult, TargetOutput, TargetState

__all__ = [
    "AlgorithmFrameResult",
    "TargetOutput",
    "TargetState",
    "TrackedRadarPipeline",
]
