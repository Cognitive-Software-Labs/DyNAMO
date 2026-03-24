from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple


class MissionState(Enum):
    INIT = auto()
    EXPLORE = auto()
    SCAN = auto()
    APPROACH_TARGET = auto()
    WAIT = auto()
    RETURN_HOME = auto()
    RECOVERY = auto()
    DONE = auto()


class ReturnSubState(Enum):
    NAVIGATE_HOME = auto()
    DIRECT_HOME = auto()
    RECOVERY = auto()
    FINAL_ALIGNMENT = auto()
    COMPLETE = auto()


class NavGoalStatus(Enum):
    IDLE = auto()
    ACTIVE = auto()
    SUCCEEDED = auto()
    FAILED = auto()


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class LaserSnapshot:
    front: float = float("inf")
    rear: float = float("inf")
    left: float = float("inf")
    right: float = float("inf")
    global_min: float = float("inf")


@dataclass
class TargetDetection:
    detected: bool = False
    stable: bool = False
    cx: Optional[float] = None
    cy: Optional[float] = None
    area: float = 0.0
    circularity: float = 0.0
    radius_px: float = 0.0
    est_distance_m: float = float("inf")


@dataclass
class ReturnUpdate:
    state: ReturnSubState
    done: bool = False
    command_linear: float = 0.0
    command_angular: float = 0.0
    require_replan: bool = False
    debug_text: str = ""


MapGoal = Tuple[float, float]
