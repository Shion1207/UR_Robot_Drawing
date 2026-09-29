import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from llm_planner.prompt import build_system_prompt, build_user_prompt  # noqa: E402
from llm_planner.student import make_student_task  # noqa: E402

R, Y, B = "red_cube", "yellow_cube", "blue_cube"


@pytest.mark.parametrize("student_id, p, a, b, c", [
    ("23020100", 0, R, Y, B),   # 00 mod 6 = 0
    ("23020101", 1, R, B, Y),
    ("23020102", 2, Y, R, B),
    ("23020103", 3, Y, B, R),
    ("23020104", 4, B, R, Y),
    ("23020105", 5, B, Y, R),
    ("23020123", 5, B, Y, R),   # vi du trong de bai: 23 mod 6 = 5
    ("23020199", 3, Y, B, R),   # 99 mod 6 = 3
])
def test_assignment_table(student_id, p, a, b, c):
    t = make_student_task("SV", student_id)
    assert t.p == p
    assert t.targets == {"zone_a": a, "zone_b": b, "zone_c": c}


def test_example_from_assignment():
    t = make_student_task("Nguyen Van An", "23020123")
    assert t.formula() == "P = 23 mod 6 = 5"
    assert t.assignment_lines() == ["zone_a <- blue_cube", "zone_b <- yellow_cube",
                                    "zone_c <- red_cube"]


def test_integer_student_id_accepted():
    # ROS doc "-p student.id:=23020123" thanh so nguyen
    assert make_student_task("SV", 23020123).p == 5


@pytest.mark.parametrize("bad", ["", "7", "23A20123", "abc"])
def test_invalid_student_id_rejected(bad):
    with pytest.raises(ValueError):
        make_student_task("SV", bad)


def test_system_prompt_contains_personal_task():
    t = make_student_task("Nguyen Van An", "23020123")
    prompt = build_system_prompt({R: "red", Y: "yellow", B: "blue"},
                                 {"zone_a": "A", "zone_b": "B", "zone_c": "C",
                                  "zone_tmp": "tmp"}, t)
    assert "MSSV 23020123" in prompt
    assert "zone_a <- blue_cube" in prompt
    assert "zone_tmp" in prompt


def test_user_prompt_summarizes_zones_without_coordinates():
    state = {"held_object": None,
             "objects": {R: {"position": [0.2, 0.2, 0.02], "in_zone": "zone_b"},
                         Y: {"position": [0.25, -0.17, 0.02], "in_zone": None},
                         B: {"position": [0.3, -0.05, 0.02], "in_zone": None}},
             "zones": ["zone_a", "zone_b", "zone_c", "zone_tmp"]}
    text = build_user_prompt("Arrange all objects according to my student ID.", state)
    assert "red_cube: nam trong zone_b" in text
    assert "Vung dang trong: zone_a, zone_c, zone_tmp" in text
    assert "0.2" not in text
