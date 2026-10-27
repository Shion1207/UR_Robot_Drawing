"""Safety contracts for the actual gesture state machine (no ROS/camera mocks)."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/assign4_bringup/scripts'))

try:
    from gesture_logic import BehaviorController, GestureFilter, WaveDetector, scan_clearances
except ImportError:
    BehaviorController = GestureFilter = WaveDetector = scan_clearances = None


class GestureSafetyTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(BehaviorController, 'Gesture behavior core is not implemented yet')
        self.c = BehaviorController()
        self.c.update_pose(0.0, 0.0, 0.0, 0.0)
        self.c.update_scan(3.0, 3.0, 3.0, 0.0)

    def start(self, gesture='Pointing_Up'):
        self.c.gesture(gesture, 0.0)
        self.c.gesture('Thumb_Up', 0.1)

    def test_selection_requires_confirmation(self):
        self.c.gesture('Pointing_Up', 0.0)
        self.assertEqual(self.c.tick(0.1), (0.0, 0.0))
        self.c.gesture('Thumb_Up', 0.1)
        self.assertGreater(self.c.tick(0.2)[0], 0.0)

    def test_thumb_without_selection_does_not_move(self):
        self.c.gesture('Thumb_Up', 0.1)
        self.assertEqual(self.c.tick(0.2), (0.0, 0.0))

    def test_open_palm_and_fist_cancel_motion_and_selection(self):
        for label in ('Open_Palm', 'Closed_Fist'):
            with self.subTest(label=label):
                self.start()
                self.c.gesture(label, 0.2)
                self.assertEqual(self.c.tick(0.2), (0.0, 0.0))
                self.c.gesture('Thumb_Up', 0.3)
                self.assertEqual(self.c.tick(0.3), (0.0, 0.0))

    def test_no_hand_stops_even_if_camera_keeps_publishing(self):
        self.start()
        for t in (0.3, 0.6, 0.9, 1.2):
            self.c.gesture('None', t)
        self.assertEqual(self.c.tick(1.2), (0.0, 0.0))

    def test_missing_stream_and_stale_sensors_stop(self):
        self.start()
        self.assertEqual(self.c.tick(2.0), (0.0, 0.0))
        self.c = BehaviorController()
        self.start()
        self.assertEqual(self.c.tick(0.2), (0.0, 0.0))

    def test_front_obstacle_aborts_and_requires_new_confirmation(self):
        self.start()
        self.c.update_scan(0.2, 3.0, 0.2, 0.2)
        self.assertEqual(self.c.tick(0.2), (0.0, 0.0))
        self.c.update_scan(3.0, 3.0, 3.0, 0.3)
        self.c.gesture('Thumb_Up', 0.3)
        self.assertEqual(self.c.tick(0.3), (0.0, 0.0))

    def test_stale_scan_stops_despite_live_hand_and_odometry(self):
        self.start()
        self.c.gesture('Thumb_Up', 4.0)
        self.c.update_pose(.01, 0, 0, 4.0)
        self.assertEqual(self.c.tick(4.0), (0.0, 0.0))
        self.assertIn('LiDAR', self.c.status)

    def test_stuck_robot_stops_instead_of_sending_velocity_forever(self):
        self.start()
        self.c.gesture('Thumb_Up', 11.0)
        self.c.update_pose(0, 0, 0, 11.0)
        self.c.update_scan(3, 3, 3, 11.0)
        self.assertEqual(self.c.tick(11.0), (0.0, 0.0))
        self.assertIn('progress', self.c.status)

    def test_greeting_alternates_turns_and_finishes(self):
        self.start('Wave')
        yaw = 0.0
        for i, delta in enumerate((.46, -.91, .91, -.91, .46)):
            t = .1+i*.1
            self.assertGreater(self.c.tick(t)[1] * delta, 0)
            yaw += delta
            self.c.update_pose(0, 0, yaw, t+.05)
            self.c.gesture('Thumb_Up', t+.05)
            self.c.tick(t+.05)
        self.assertIsNone(self.c.active)

    def test_four_forward_backward_cycles_and_no_held_thumb_retrigger(self):
        self.start()
        x, now = 0.0, 0.1
        for phase in range(8):
            self.assertEqual(self.c.active, 'forward_back')
            direction = 1 if phase % 2 == 0 else -1
            self.assertGreater(self.c.tick(now)[0] * direction, 0)
            x += direction * 0.12
            now += 0.1
            self.c.gesture('Thumb_Up', now)
            self.c.update_scan(3.0, 3.0, 3.0, now)
            self.c.update_pose(x, 0.0, 0.0, now)
            self.c.tick(now)
        self.assertIsNone(self.c.active)
        self.c.gesture('Thumb_Up', now + 0.1)
        self.assertEqual(self.c.tick(now + 0.1), (0.0, 0.0))

    def test_reverse_obstacle_stops_backward_phase(self):
        self.start()
        self.c.update_pose(0.25, 0, 0, 0.2)
        self.c.tick(0.2)
        self.c.update_scan(3.0, 0.2, 0.2, 0.3)
        self.assertEqual(self.c.tick(0.3), (0.0, 0.0))

    def test_circle_requires_confirmation_and_completes_full_turn(self):
        self.start('Victory')
        v, w = self.c.tick(0.1)
        self.assertGreater(v, 0)
        self.assertGreater(w, 0)
        for i in range(1, 33):
            t = 0.1 + i * 0.05
            yaw = i * 0.2
            self.c.gesture('Thumb_Up', t)
            self.c.update_scan(3, 3, 3, t)
            self.c.update_pose(0.3 * math.sin(yaw), 0.3 * (1-math.cos(yaw)),
                               math.atan2(math.sin(yaw), math.cos(yaw)), t)
            self.c.tick(t)
        self.assertIsNone(self.c.active)

    def test_wave_also_requires_confirmation(self):
        self.c.gesture('Wave', 0)
        self.assertEqual(self.c.tick(0.1), (0.0, 0.0))
        self.c.gesture('Thumb_Up', 0.1)
        self.assertNotEqual(self.c.tick(0.2)[1], 0)


class RecognitionTests(unittest.TestCase):
    def test_wave_requires_two_direction_changes_not_static_palm(self):
        self.assertIsNotNone(WaveDetector, 'Wave detector is not implemented yet')
        detector = WaveDetector()
        for i in range(12):
            self.assertFalse(detector.update('Open_Palm', 0.5, i * 0.1))
        found = [detector.update('Open_Palm', x, 1.3+i*.1)
                 for i, x in enumerate([.5,.7,.45,.72,.43])]
        self.assertTrue(any(found))

    def test_debounce_rejects_flicker_but_stop_is_immediate(self):
        self.assertIsNotNone(GestureFilter, 'Gesture filter is not implemented yet')
        f = GestureFilter(hold_seconds=0.25)
        self.assertEqual(f.update('Pointing_Up', 0), 'None')
        self.assertEqual(f.update('Victory', .1), 'None')
        self.assertEqual(f.update('Victory', .4), 'Victory')
        self.assertEqual(f.update('Open_Palm', .41), 'Open_Palm')

    def test_invalid_scan_is_not_clear_but_positive_infinity_is(self):
        self.assertIsNotNone(scan_clearances, 'Scan parser is not implemented yet')
        self.assertEqual(scan_clearances([float('nan')] * 360, -math.pi,
                                        math.pi/180, .1, 10), (0.0, 0.0, 0.0))
        self.assertEqual(scan_clearances([float('inf')] * 360, -math.pi,
                                        math.pi/180, .1, 10), (10, 10, 10))

    def test_malformed_scan_metadata_blocks_rotation_too(self):
        self.assertIsNotNone(scan_clearances, 'Scan parser is not implemented yet')
        self.assertEqual(scan_clearances([5.0]*360, float('nan'), .01, .1, 10),
                         (0.0, 0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
