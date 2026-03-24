from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from ridgeback_vision_detection.mission_types import TargetDetection


@dataclass
class PerceptionConfig:
    image_width: int = 640
    camera_hfov_rad: float = 1.047
    sphere_radius_m: float = 0.30
    min_area_px: int = 120
    min_circularity: float = 0.55
    stable_history: int = 6
    stable_required: int = 4


class PerceptionManager:
    """Vision-only target detection; no mission decisions."""

    def __init__(self, config: PerceptionConfig | None = None) -> None:
        self.cfg = config or PerceptionConfig()
        self.bridge = CvBridge()
        self.last_detection = TargetDetection()
        self._history = deque(maxlen=self.cfg.stable_history)
        self.last_annotated = None
        self._last_seen_time = 0.0

    def _estimate_distance(self, radius_px: float) -> float:
        if radius_px <= 0.0:
            return float("inf")
        focal_px = (self.cfg.image_width / 2.0) / np.tan(self.cfg.camera_hfov_rad / 2.0)
        diameter_px = 2.0 * radius_px
        return (2.0 * self.cfg.sphere_radius_m * focal_px) / max(1e-3, diameter_px)

    def update_from_image(self, msg: Image, now_sec: float) -> TargetDetection:
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lo1, hi1 = np.array([0, 50, 40]), np.array([10, 255, 255])
        lo2, hi2 = np.array([160, 50, 40]), np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lo1, hi1) | cv2.inRange(hsv, lo2, hi2)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detection = TargetDetection()

        if contours:
            cnt = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(cnt))
            peri = float(cv2.arcLength(cnt, True))
            circularity = (4.0 * np.pi * area / (peri * peri)) if peri > 1e-6 else 0.0

            if area >= self.cfg.min_area_px and circularity >= self.cfg.min_circularity:
                (cx, cy), radius = cv2.minEnclosingCircle(cnt)
                detection.detected = True
                detection.cx = cx
                detection.cy = cy
                detection.area = area
                detection.circularity = circularity
                detection.radius_px = radius
                detection.est_distance_m = self._estimate_distance(radius)
                self._last_seen_time = now_sec

                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 255), 2)

        self._history.append(1 if detection.detected else 0)
        detection.stable = sum(self._history) >= self.cfg.stable_required

        self.last_detection = detection
        self.last_annotated = frame
        return detection

    def seconds_since_seen(self, now_sec: float) -> float:
        if self._last_seen_time <= 0.0:
            return float("inf")
        return now_sec - self._last_seen_time
