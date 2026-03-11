from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class PoseEstimate:
    x: float
    y: float
    z: float
    yaw: float


@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass
class Detection2D:
    u: float
    v: float
    conf: float
    bbox: Tuple[float, float, float, float]
    cls_id: Optional[int] = None
    cls_name: str = ""
    track_id: Optional[int] = None
    track_hits: int = 0
    track_age_s: float = 0.0
    track_state: str = "raw"
    track_switched: bool = False


@dataclass
class ProjectedTarget:
    pose: PoseEstimate
    range_m: float
    range_source: str
    bearing_rad: float
