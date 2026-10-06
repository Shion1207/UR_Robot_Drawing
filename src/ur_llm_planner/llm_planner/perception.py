"""Cac ham perception thuan Python, tach khoi ROS de de kiem thu.

Camera nhin vuong goc tu tren xuong. Anh duoc phan doan theo HSV; tam blob cua moi mau
duoc chieu len mat phang mat tren cua cube bang mo hinh pinhole va extrinsic camera co dinh.
"""

from dataclasses import dataclass
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    position: Tuple[float, float, float]


# OpenCV hue nam trong [0, 179]. Moi mau co the co nhieu khoang (red bi tach o bien 0/179).
HSV_RANGES: Mapping[str, Tuple[Tuple[Tuple[int, int, int], Tuple[int, int, int]], ...]] = {
    "red": (((0, 110, 70), (9, 255, 255)), ((170, 110, 70), (179, 255, 255))),
    "yellow": (((20, 100, 80), (38, 255, 255)),),
    "green": (((42, 80, 55), (88, 255, 255)),),
    "blue": (((92, 90, 55), (132, 255, 255)),),
    "purple": (((133, 75, 55), (169, 255, 255)),),
}


def color_mask(hsv: np.ndarray, color: str) -> np.ndarray:
    """Tra ve mask nhi phan cua mot ten mau ho tro."""
    ranges = HSV_RANGES.get(color.lower())
    if not ranges:
        raise ValueError(f"Mau perception khong duoc ho tro: {color}")
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for low, high in ranges:
        mask = cv2.bitwise_or(
            mask,
            cv2.inRange(hsv, np.asarray(low, dtype=np.uint8),
                        np.asarray(high, dtype=np.uint8)),
        )
    kernel = np.ones((3, 3), dtype=np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def detect_colored_objects(
    bgr: np.ndarray,
    object_colors: Mapping[str, str],
    min_area: int = 120,
    max_area: int = 1800,
) -> Dict[str, Tuple[float, float, int]]:
    """Tim blob mau cube; ket qua la object -> (u, v, dien tich pixel).

    Gioi han dien tich loai bo nhieu mau va cac tam zone lon hon cube. Neu co nhieu blob cung
    mau, chon blob lon nhat nam trong khoang cho phep.
    """
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("Anh BGR phai co shape HxWx3")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    result: Dict[str, Tuple[float, float, int]] = {}
    for name, color in object_colors.items():
        mask = color_mask(hsv, color)
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        candidates = [
            (int(stats[i, cv2.CC_STAT_AREA]), centroids[i])
            for i in range(1, count)
            if min_area <= int(stats[i, cv2.CC_STAT_AREA]) <= max_area
        ]
        if candidates:
            area, center = max(candidates, key=lambda item: item[0])
            result[name] = (float(center[0]), float(center[1]), area)
    return result


def pixel_to_world(
    u: float,
    v: float,
    camera: CameraModel,
    plane_z: float,
) -> Tuple[float, float, float]:
    """Chieu pixel len mat phang ngang ``z=plane_z``.

    Extrinsic cua overhead camera trong world SDF:
      optical +z -> world -z, optical +x (anh sang phai) -> world -y,
      optical +y (anh di xuong) -> world -x.
    """
    if camera.fx <= 0.0 or camera.fy <= 0.0:
        raise ValueError("Camera focal length phai duong")
    height = camera.position[2] - plane_z
    if height <= 0.0:
        raise ValueError("Mat phang chieu phai nam duoi camera")
    x = camera.position[0] - height * (v - camera.cy) / camera.fy
    y = camera.position[1] - height * (u - camera.cx) / camera.fx
    return x, y, plane_z


def zone_of(
    position: Sequence[float],
    zones: Mapping[str, Sequence[float]],
    zone_size: float,
    tolerance: float = 0.01,
) -> Optional[str]:
    """Xac dinh position co nam trong zone nao khong."""
    half = zone_size / 2.0 + tolerance
    for name, center in zones.items():
        if abs(position[0] - center[0]) <= half and abs(position[1] - center[1]) <= half:
            return name
    return None


def scene_from_detections(
    positions: Mapping[str, Sequence[float]],
    visible: Iterable[str],
    zones: Mapping[str, Sequence[float]],
    zone_size: float,
) -> dict:
    """Tao state JSON gon cho quan sat/debug truc tiep tu camera."""
    visible_set = set(visible)
    objects = {
        name: {
            "position": [round(float(v), 4) for v in position],
            "in_zone": zone_of(position, zones, zone_size),
            "visible": name in visible_set,
        }
        for name, position in positions.items()
    }
    occupied = {
        zone: next((name for name, info in objects.items() if info["in_zone"] == zone), None)
        for zone in zones
    }
    return {
        "source": "camera",
        "camera_ready": bool(positions) and set(positions).issubset(visible_set),
        "objects": objects,
        "zones": occupied,
    }
