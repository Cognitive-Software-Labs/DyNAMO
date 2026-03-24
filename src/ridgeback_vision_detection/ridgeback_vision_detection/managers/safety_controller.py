from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Twist

from ridgeback_vision_detection.mission_types import LaserSnapshot


@dataclass
class SafetyConfig:
    min_front_stop: float = 0.45
    min_side_stop: float = 0.32
    min_rear_stop: float = 0.30
    critical_stop: float = 0.25
    max_speed: float = 0.30
    max_yaw: float = 1.20


class SafetyController:
    """Single point for velocity safety and smoothing constraints."""

    def __init__(self, config: SafetyConfig | None = None) -> None:
        self.cfg = config or SafetyConfig()

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    @staticmethod
    def _scale(value: float, threshold: float, clear: float) -> float:
        if value <= threshold:
            return 0.0
        if value >= clear:
            return 1.0
        return (value - threshold) / max(1e-6, (clear - threshold))

    def apply(self, desired: Twist, scan: LaserSnapshot, allow_spin: bool = False) -> Twist:
        out = Twist()

        # Clamp requested command first.
        out.linear.x = self._clamp(desired.linear.x, -self.cfg.max_speed, self.cfg.max_speed)
        out.angular.z = self._clamp(desired.angular.z, -self.cfg.max_yaw, self.cfg.max_yaw)

        # Critical safety stop with optional low-speed spin.
        if scan.global_min < self.cfg.critical_stop:
            out.linear.x = 0.0
            out.angular.z = out.angular.z if allow_spin else 0.0
            out.angular.z = self._clamp(out.angular.z, -0.45, 0.45)
            return out

        if out.linear.x > 0.0:
            near_front = scan.front <= self.cfg.min_front_stop
            near_side = min(scan.left, scan.right) <= (self.cfg.min_side_stop + 0.02)
            if near_front or near_side:
                out.linear.x = 0.0
                if scan.left < scan.right:
                    out.angular.z += 0.35
                elif scan.right < scan.left:
                    out.angular.z -= 0.35

            front_scale = self._scale(scan.front, self.cfg.min_front_stop, self.cfg.min_front_stop + 0.6)
            side_scale = self._scale(min(scan.left, scan.right), self.cfg.min_side_stop, self.cfg.min_side_stop + 0.4)
            out.linear.x *= min(front_scale, side_scale)

            # Corridor centering assist with hysteresis-like deadband to avoid
            # left-right jitter from small side-range noise.
            if out.linear.x > 0.05 and not allow_spin:
                side_error = scan.right - scan.left
                if abs(side_error) > 0.08:
                    out.angular.z += self._clamp(0.9 * side_error, -0.22, 0.22)

            out.angular.z = self._clamp(out.angular.z, -self.cfg.max_yaw, self.cfg.max_yaw)

        elif out.linear.x < 0.0:
            rear_scale = self._scale(scan.rear, self.cfg.min_rear_stop, self.cfg.min_rear_stop + 0.5)
            out.linear.x *= rear_scale

        return out
