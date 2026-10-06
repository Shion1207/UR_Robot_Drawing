import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from llm_planner.perception import (  # noqa: E402
    CameraModel,
    detect_colored_objects,
    pixel_to_world,
    scene_from_detections,
    zone_of,
)


def test_detects_five_colored_blocks():
    image = np.zeros((300, 300, 3), dtype=np.uint8)
    colors = {
        "red_cube": (0, 0, 230),
        "yellow_cube": (0, 230, 230),
        "blue_cube": (230, 40, 20),
        "green_cube": (20, 210, 40),
        "purple_cube": (210, 30, 160),
    }
    names = {}
    for i, (name, bgr) in enumerate(colors.items()):
        x, y = 25 + i * 52, 120
        cv2.rectangle(image, (x, y), (x + 24, y + 24), bgr, -1)
        names[name] = name.removesuffix("_cube")

    result = detect_colored_objects(image, names, min_area=300, max_area=900)
    assert set(result) == set(colors)
    assert abs(result["red_cube"][1] - 132.0) < 1.0


def test_pixel_to_world_overhead_center_and_axes():
    camera = CameraModel(500.0, 500.0, 400.0, 400.0, (0.15, 0.0, 1.2))
    assert pixel_to_world(400, 400, camera, 0.04) == (0.15, 0.0, 0.04)
    right = pixel_to_world(450, 400, camera, 0.04)
    down = pixel_to_world(400, 450, camera, 0.04)
    assert right[1] < 0.0
    assert down[0] < 0.15


def test_zone_state_from_camera_positions():
    zones = {"zone_a": (0.28, 0.10, 0.0), "zone_b": (0.21, 0.21, 0.0)}
    positions = {"red_cube": (0.281, 0.099, 0.02), "blue_cube": (0.3, -0.1, 0.02)}
    assert zone_of(positions["red_cube"], zones, 0.08) == "zone_a"
    assert zone_of(positions["blue_cube"], zones, 0.08) is None

    state = scene_from_detections(positions, positions, zones, 0.08)
    assert state["source"] == "camera"
    assert state["objects"]["red_cube"]["in_zone"] == "zone_a"
    assert state["zones"] == {"zone_a": "red_cube", "zone_b": None}
