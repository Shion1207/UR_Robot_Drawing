"""Image-result filtering and finite, sensor-gated motions independent of ROS/AI."""
import math

LABELS = {'happy', 'sad', 'neutral', 'angry', 'surprise', 'fear'}


def classify_result(results):
    count = len(results) if isinstance(results, list) else 0
    if count != 1:
        return 'none', 0., count
    item = results[0]
    label = str(item.get('dominant_emotion', 'none')).lower()
    region = item.get('region', {})
    if region.get('w', 0) <= 0 or region.get('h', 0) <= 0:
        return 'none', 0., count
    try:
        score = float(item.get('emotion', {}).get(label, 0.))/100.
    except (TypeError, ValueError):
        score = 0.
    if label not in LABELS or not math.isfinite(score) or not 0 <= score <= 1:
        return 'none', 0., count
    return label, score, count


class EmotionFilter:
    def __init__(self, hold=.6, confidence=.60):
        if hold < 0 or not 0 < confidence <= 1:
            raise ValueError('Invalid filter thresholds')
        self.hold, self.confidence = hold, confidence
        self.reset()

    def reset(self):
        self.candidate, self.since = 'none', None

    def update(self, label, score, faces, now):
        if label not in LABELS or faces != 1 or not math.isfinite(score) or score < self.confidence:
            self.reset()
            return 'none'
        if label != self.candidate:
            self.candidate, self.since = label, now
        return label if now-self.since >= self.hold else 'pending'


def scan_clearances(ranges, angle_min, angle_increment, range_min, range_max, yaw_offset=0., sensor_offset=0.):
    """Conservative min front/rear/all; missing or invalid sectors block motion."""
    if not all(math.isfinite(v) for v in (angle_min, angle_increment, range_min, range_max)) or range_max <= range_min:
        return 0., 0., 0.
    front, rear, around = [], [], []
    invalid = False
    for i, value in enumerate(ranges):
        a = math.atan2(math.sin(angle_min+i*angle_increment+yaw_offset), math.cos(angle_min+i*angle_increment+yaw_offset))
        if value == math.inf:
            value = range_max
        elif not math.isfinite(value) or not range_min <= value <= range_max:
            invalid = True
            value = 0.
        value=max(0.,value-sensor_offset)
        around.append(value)
        if abs(a) <= math.pi/4:front.append(value)
        if abs(a) >= 3*math.pi/4:rear.append(value)
    # An incomplete front-only scan cannot establish rear/rotation clearance.
    complete=abs(angle_increment)*max(0,len(ranges)-1)>=2*math.pi-3*abs(angle_increment)
    return min(front, default=0.), min(rear, default=0.), (0. if invalid or not front or not rear or not complete else min(around,default=0.))


class MotionController:
    def __init__(self, linear_speed=.12, angular_speed=.45, step_distance=.15,
                 arc_distance=.35, arc_radius=.60, emotion_timeout=1.5,
                 sensor_timeout=2.5, stop_distance=.50, rotation_clearance=.35,
                 progress_timeout=10., phase_timeout=180., rearm_seconds=.4):
        values=locals().copy();values.pop('self')
        if any(not math.isfinite(v) or v <= 0 for v in values.values()):
            raise ValueError('Motion parameters must be positive finite numbers')
        self.__dict__.update(values)
        self.status='WAIT_NEUTRAL: keep a neutral face to arm'
        self.armed=False
        self.active=None
        self.neutral_since=None
        self.last_emotion=None
        self.last_pose=None
        self.last_scan=None
        self.position=None
        self.front=self.rear=self.around=0.
        self.distance=self.turn=0.
        self.started=self.progress_at=0.
        self.progress_mark=0.

    def stop(self, reason):
        self.active=None
        self.armed=False
        self.neutral_since=None
        self.status='STOP: '+reason+'; neutral to rearm'

    def emotion(self,label,now):
        if label=='pending':
            # Valid face, but the new label is still settling. Preserve an idle
            # READY latch; unlike missing/uncertain faces this is not loss.
            self.neutral_since=None
            if self.active:self.stop('expression transition')
            return
        if label not in LABELS:
            self.stop('face missing/uncertain or unsupported expression')
            return
        self.last_emotion=now
        if label=='neutral':
            if self.active:
                self.stop('neutral')
            if self.neutral_since is None:self.neutral_since=now
            if now-self.neutral_since >= self.rearm_seconds:
                self.armed=True
                self.status='READY: choose one expression'
            return
        self.neutral_since=None
        if self.active:
            if label != self.active:self.stop('expression changed')
            return
        if not self.armed:return
        self.armed=False
        self.active=label
        self.distance=self.turn=self.progress_mark=0.
        self.started=self.progress_at=now
        self.status='RUNNING: '+label

    def pose(self,x,y,yaw,now):
        if not all(math.isfinite(v) for v in (x,y,yaw)):
            self.last_pose=None;self.stop('invalid odometry');return
        if self.position is not None:
            px,py,pa=self.position
            distance=math.hypot(x-px,y-py)
            turn=math.atan2(math.sin(yaw-pa),math.cos(yaw-pa))
            if self.active:
                if distance > .8 or abs(turn) > .8:self.stop('odometry jump')
                else:self.distance+=distance;self.turn+=turn
        self.position=(x,y,yaw)
        self.last_pose=now

    def scan(self,front,rear,around,now):
        self.front,self.rear,self.around=(v if math.isfinite(v) and v>=0 else 0. for v in (front,rear,around))
        self.last_scan=now

    def tick(self,now):
        zero=(0.,0.)
        if not self.active:
            if self.armed and (self.last_emotion is None or now-self.last_emotion>self.emotion_timeout):
                self.stop('expression timeout while ready')
            return zero
        if self.last_emotion is None or now-self.last_emotion>self.emotion_timeout:
            self.stop('expression timeout');return zero
        if any(t is None or now-t>self.sensor_timeout for t in (self.last_pose,self.last_scan)):
            self.stop('sensors not ready/stale');return zero
        label=self.active
        progress=self.turn if label=='surprise' else self.distance
        target=2*math.pi if label=='surprise' else self.arc_distance if label in ('happy','sad') else self.step_distance
        if progress>=target:
            self.stop('DONE '+label);return zero
        if progress-self.progress_mark>=.02:
            self.progress_at,self.progress_mark=now,progress
        if now-self.progress_at>self.progress_timeout or now-self.started>self.phase_timeout:
            self.stop('no progress/time limit');return zero
        v,w=self.linear_speed,0.
        if label=='angry':v=-v
        if label in ('happy','sad'):w=v/self.arc_radius*(1 if label=='happy' else -1)
        if label=='surprise':v,w=0.,self.angular_speed
        if ((v>0 and self.front<self.stop_distance) or
            (v<0 and self.rear<self.stop_distance) or
            (w!=0 and self.around<self.rotation_clearance)):
            self.stop('obstacle/unknown scan sector');return zero
        return v,w


class DirectMotionController(MotionController):
    """Direct expression control, without the previous Neutral/READY selection gate."""
    def __init__(self,**options):
        super().__init__(**options)
        self.blocked_label=None
        self.status='WAIT_FACE: show an expression'

    def stop(self,reason):
        if self.active:self.blocked_label=self.active
        super().stop(reason)
        self.status='STOP: '+reason

    def emotion(self,label,now):
        if label=='pending':
            if self.active:self.stop('expression changing')
            return
        if label not in LABELS:
            self.stop('face missing/uncertain')
            return
        self.last_emotion=now
        if label=='neutral':
            self.stop('neutral');self.blocked_label=None
            return
        if self.active==label:return
        if self.blocked_label==label:return
        if self.active:self.stop('new expression')
        self.blocked_label=None
        self.active=label
        self.distance=self.turn=self.progress_mark=0.
        self.started=self.progress_at=now
        self.status='RUNNING: '+label

    def tick(self,now):
        # Happy/sad follow the face continuously; step motions and one full turn
        # remain finite. All sensor, obstacle and time-limit checks still apply.
        previous=self.arc_distance
        self.arc_distance=math.inf
        try:return super().tick(now)
        finally:self.arc_distance=previous
