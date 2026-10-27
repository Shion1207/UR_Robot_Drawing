"""ROS-independent recognition filters and fail-safe gesture behavior controller."""
import math
from dataclasses import dataclass

LABELS = {'Wave', 'Pointing_Up', 'Victory', 'Thumb_Up', 'Open_Palm', 'Closed_Fist'}
SELECTIONS = {'Wave': 'greeting', 'Pointing_Up': 'forward_back', 'Victory': 'circle'}


class GestureFilter:
    """Require a stable label, but never delay a stop/cancel observation."""
    def __init__(self, hold_seconds=0.25):
        self.hold_seconds = hold_seconds
        self.candidate, self.since = 'None', 0.0

    def update(self, label, now):
        if label not in LABELS:
            label = 'None'
        if label != self.candidate:
            self.candidate, self.since = label, now
        if label in ('Open_Palm', 'Closed_Fist', 'None'):
            return label
        return label if now - self.since >= self.hold_seconds else 'None'


class WaveDetector:
    """A palm must travel horizontally and reverse at least twice in a short window."""
    def __init__(self, excursion=0.08, window=1.8, latch=0.7):
        self.excursion, self.window, self.latch = excursion, window, latch
        self.anchor = None
        self.direction = self.reversals = 0
        self.started = 0.0
        self.latched_until = -1.0

    def update(self, label, wrist_x, now):
        if label != 'Open_Palm' or wrist_x is None:
            self.anchor, self.direction, self.reversals = None, 0, 0
            self.latched_until = -1.0
            return False
        if now < self.latched_until:
            return True
        if self.anchor is None or now - self.started > self.window:
            self.anchor, self.started = wrist_x, now
            self.direction, self.reversals = 0, 0
            return False
        delta = wrist_x - self.anchor
        if abs(delta) < self.excursion:
            return False
        direction = 1 if delta > 0 else -1
        if self.direction and direction != self.direction:
            self.reversals += 1
        self.direction, self.anchor = direction, wrist_x
        if self.reversals >= 2:
            self.latched_until = now + self.latch
            self.anchor = None
            return True
        return False


def scan_clearances(ranges, angle_min, angle_increment, range_min, range_max):
    """Front/rear/all clearance; invalid or uncovered sectors are unsafe (zero)."""
    if (not all(math.isfinite(v) for v in (angle_min, angle_increment, range_min, range_max))
            or angle_increment <= 0 or range_min < 0 or range_max <= range_min):
        return 0.0, 0.0, 0.0
    front, rear, all_points = [], [], []
    for i, value in enumerate(ranges):
        if value == math.inf:
            value = range_max  # LaserScan: valid no-return beam.
        elif not math.isfinite(value) or value < range_min or value > range_max:
            continue
        angle = math.atan2(math.sin(angle_min + i * angle_increment),
                           math.cos(angle_min + i * angle_increment))
        all_points.append(value)
        if abs(angle) <= math.radians(40):
            front.append(value)
        if abs(angle) >= math.radians(140):
            rear.append(value)
    return (min(front, default=0.0), min(rear, default=0.0),
            min(all_points, default=0.0))


@dataclass(frozen=True)
class Motion:
    linear: float
    angular: float
    distance: float = 0.0
    angle: float = 0.0


class BehaviorController:
    """Select -> confirm -> execute once; stop always clears pending commands."""
    def __init__(self, linear_speed=0.15, angular_speed=0.55, step_distance=0.10,
                 circle_radius=0.40, gesture_timeout=0.9, sensor_timeout=3.0,
                 selection_timeout=8.0, phase_timeout=240.0, progress_timeout=10.0,
                 stop_distance=0.50, rotation_clearance=0.30):
        values = locals().copy()
        values.pop('self')
        for name, value in values.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, value)
        self.selected = self.active = None
        self.status = 'IDLE: choose gesture, then Thumb_Up'
        self.last_label = 'None'
        self.last_control = self.selected_at = -math.inf
        self.pose_at = self.scan_at = -math.inf
        self.pose = None
        self.clearances = (0.0, 0.0, 0.0)
        self.phases = []
        self.phase_index = 0
        self.distance = self.rotation = 0.0
        self.phase_started = 0.0
        self.progress_at = 0.0

    def stop(self, reason):
        self.selected = self.active = None
        self.phases = []
        self.status = reason

    def update_pose(self, x, y, yaw, now):
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return
        if self.pose and self.active:
            px, py, old_yaw = self.pose
            # Do not count a simulator reset / teleported robot as progress.
            if math.hypot(x-px, y-py) > 0.5:
                self.stop('STOP: odometry jump/reset')
            else:
                delta_distance = math.hypot(x-px, y-py)
                delta_yaw = math.atan2(math.sin(yaw-old_yaw), math.cos(yaw-old_yaw))
                self.distance += delta_distance
                self.rotation += delta_yaw
                if delta_distance > 0.0001 or abs(delta_yaw) > 0.001:
                    self.progress_at = now
        self.pose, self.pose_at = (x, y, yaw), now

    def update_scan(self, front, rear, nearest, now):
        self.clearances = front, rear, nearest
        self.scan_at = now

    def gesture(self, label, now):
        if label in LABELS:
            self.last_control = now
        changed = label != self.last_label
        self.last_label = label
        if label in ('Open_Palm', 'Closed_Fist'):
            self.stop('STOP: open palm' if label == 'Open_Palm' else 'IDLE: cancelled by fist')
            return
        if self.active or not changed:
            return
        if label in SELECTIONS:
            self.selected, self.selected_at = SELECTIONS[label], now
            self.status = f'SELECTED: {self.selected}; show Thumb_Up to confirm'
        elif label == 'Thumb_Up' and self.selected:
            if now - self.selected_at > self.selection_timeout:
                self.stop('IDLE: selection expired')
                return
            self.active, self.selected = self.selected, None
            v, w = self.linear_speed, self.angular_speed
            if self.active == 'forward_back':
                self.phases = [Motion(direction*v, 0, self.step_distance)
                               for _ in range(4) for direction in (1, -1)]
            elif self.active == 'circle':
                self.phases = [Motion(v, v/self.circle_radius, angle=2*math.pi)]
            else:
                self.phases = [Motion(0, math.copysign(w, a), angle=a)
                               for a in (0.45, -0.90, 0.90, -0.90, 0.45)]
            self.phase_index, self.distance, self.rotation = 0, 0.0, 0.0
            self.phase_started = now
            self.progress_at = now
            self.status = f'RUNNING: {self.active}'

    def tick(self, now):
        zero = (0.0, 0.0)
        if now - self.last_control > self.gesture_timeout:
            if self.active or self.selected:
                self.stop('STOP: lost hand / control heartbeat')
            return zero
        if not self.active:
            if self.selected and now - self.selected_at > self.selection_timeout:
                self.stop('IDLE: selection expired')
            return zero
        if now - self.pose_at > self.sensor_timeout or now - self.scan_at > self.sensor_timeout:
            self.stop('STOP: odometry / LiDAR not ready or stale')
            return zero
        if now - self.phase_started > self.phase_timeout:
            self.stop('STOP: motion phase timed out')
            return zero
        if now - self.progress_at > self.progress_timeout:
            self.stop('STOP: no motion progress; check Create 3 safety / simulator')
            return zero
        phase = self.phases[self.phase_index]
        completed = ((phase.distance > 0 and self.distance >= phase.distance) or
                     (phase.angle != 0 and self.rotation * math.copysign(1, phase.angle)
                      >= abs(phase.angle)))
        if completed:
            self.phase_index += 1
            self.distance = self.rotation = 0.0
            self.phase_started = now
            if self.phase_index == len(self.phases):
                self.stop('DONE: behavior completed; choose a new gesture')
                return zero
            phase = self.phases[self.phase_index]
        front, rear, nearest = self.clearances
        if ((phase.linear > 0 and front < self.stop_distance) or
            (phase.linear < 0 and rear < self.stop_distance) or
            (phase.angular != 0 and nearest < self.rotation_clearance)):
            self.stop('STOP: obstacle too close; choose and confirm again')
            return zero
        self.status = f'RUNNING: {self.active} ({self.phase_index+1}/{len(self.phases)})'
        return phase.linear, phase.angular
