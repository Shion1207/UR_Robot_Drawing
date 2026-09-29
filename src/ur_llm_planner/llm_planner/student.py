"""Ca nhan hoa nhiem vu theo ma so sinh vien (MSSV).

P = XX mod 6, voi XX la 2 chu so cuoi cua MSSV. P quyet dinh vat nao phai nam o zone A/B/C:

    P = 0 -> A = red,    B = yellow, C = blue
    P = 1 -> A = red,    B = blue,   C = yellow
    P = 2 -> A = yellow, B = red,    C = blue
    P = 3 -> A = yellow, B = blue,   C = red
    P = 4 -> A = blue,   B = red,    C = yellow
    P = 5 -> A = blue,   B = yellow, C = red
"""

from dataclasses import dataclass
from typing import Dict, Tuple

TASK_ZONES: Tuple[str, str, str] = ("zone_a", "zone_b", "zone_c")

# P -> (vat o zone_a, vat o zone_b, vat o zone_c)
ASSIGNMENTS: Dict[int, Tuple[str, str, str]] = {
    0: ("red_cube", "yellow_cube", "blue_cube"),
    1: ("red_cube", "blue_cube", "yellow_cube"),
    2: ("yellow_cube", "red_cube", "blue_cube"),
    3: ("yellow_cube", "blue_cube", "red_cube"),
    4: ("blue_cube", "red_cube", "yellow_cube"),
    5: ("blue_cube", "yellow_cube", "red_cube"),
}


@dataclass(frozen=True)
class StudentTask:
    name: str
    student_id: str
    xx: int
    p: int
    # zone -> vat phai dat vao zone do
    targets: Dict[str, str]

    def formula(self) -> str:
        return f"P = {self.xx:02d} mod 6 = {self.p}"

    def assignment_lines(self):
        return [f"{zone} <- {obj}" for zone, obj in self.targets.items()]


def make_student_task(name: str, student_id) -> StudentTask:
    # MSSV co the duoc ROS doc thanh so nguyen (vd: -p student.id:=23020123) -> ep ve chuoi
    sid = str(student_id).strip()
    if not sid.isdigit() or len(sid) < 2:
        raise ValueError(f"MSSV khong hop le: '{sid}' (phai la chuoi chu so, it nhat 2 chu so)")
    xx = int(sid[-2:])
    p = xx % 6
    return StudentTask(name=str(name).strip(), student_id=sid, xx=xx, p=p,
                       targets=dict(zip(TASK_ZONES, ASSIGNMENTS[p])))
