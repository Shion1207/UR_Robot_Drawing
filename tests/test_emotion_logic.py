import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/assign8_bringup/scripts'))
from emotion_logic import EmotionFilter, MotionController, classify_result, scan_clearances


class EmotionTests(unittest.TestCase):
    def test_six_labels_and_disgust_not_a_command(self):
        f = EmotionFilter(hold=.4)
        for label in ['happy','sad','neutral','angry','surprise','fear']:
            f.reset()
            self.assertEqual(f.update(label,.9,1,0), 'pending')
            self.assertEqual(f.update(label,.9,1,.5), label)
        self.assertEqual(f.update('disgust',.99,1,.6),'none')

    def test_low_confidence_multiple_faces_and_jitter(self):
        f=EmotionFilter(hold=.4)
        for label,score,count,t in [('happy',.4,1,0),('happy',.9,2,1),('happy',.9,0,2)]:
            self.assertEqual(f.update(label,score,count,t),'none')
        self.assertEqual(f.update('happy',.9,1,3),'pending')
        self.assertEqual(f.update('sad',.9,1,3.3),'pending')
        self.assertEqual(f.update('sad',.9,1,3.8),'sad')
        self.assertEqual(f.update('happy',float('nan'),1,4),'none')

    def test_deepface_result_requires_single_real_face(self):
        result=[{'dominant_emotion':'happy','emotion':{'happy':95},'region':{'x':0,'y':0,'w':30,'h':30}}]
        self.assertEqual(classify_result(result),('happy',.95,1))
        self.assertEqual(classify_result(result*2)[0],'none')
        self.assertEqual(classify_result([]),('none',0.,0))


class MotionTests(unittest.TestCase):
    def test_real_filter_to_controller_transition_can_start_but_loss_disarms(self):
        c=MotionController();f=EmotionFilter(hold=.4)
        for label,t in [('neutral',0),('neutral',.5),('neutral',1.),('happy',1.1),('happy',1.6)]:
            self.fresh(c,t)
            c.emotion(f.update(label,.95,1,t),t)
            command=c.tick(t)
        self.assertGreater(command[0],0)
        self.assertGreater(command[1],0)
        c.emotion(f.update('happy',.1,1,1.7),1.7)
        self.assertEqual(c.tick(1.7),(0.,0.))
        self.assertFalse(c.armed)

    def ready(self):
        c=MotionController()
        self.fresh(c,0)
        c.emotion('neutral',0)
        c.emotion('neutral',.5)
        return c

    def fresh(self,c,t,x=0.,y=0.,yaw=0.,clearance=5.):
        c.pose(x,y,yaw,t)
        c.scan(clearance,clearance,clearance,t)

    def test_requires_neutral_before_first_motion(self):
        c=MotionController(); self.fresh(c,0)
        c.emotion('happy',0)
        self.assertEqual(c.tick(0),(0.,0.))

    def test_all_six_mappings(self):
        for label,sign_v,sign_w in [('happy',1,1),('sad',1,-1),('angry',-1,0),('surprise',0,1),('fear',1,0)]:
            c=self.ready(); self.fresh(c,.6);c.emotion(label,.6)
            v,w=c.tick(.6)
            self.assertEqual((math.copysign(1,v) if v else 0,math.copysign(1,w) if w else 0),(sign_v,sign_w))
            c.emotion('neutral',.7);self.assertEqual(c.tick(.7),(0.,0.))

    def test_finite_distance_and_held_label_does_not_repeat(self):
        c=self.ready();self.fresh(c,.6);c.emotion('fear',.6)
        self.assertGreater(c.tick(.6)[0],0)
        self.fresh(c,.7,x=.16)
        self.assertEqual(c.tick(.7),(0.,0.))
        c.emotion('fear',.8); self.assertEqual(c.tick(.8),(0.,0.))

    def test_full_turn_unwraps_pi_and_not_just_a_timer(self):
        c=self.ready();self.fresh(c,.6);c.emotion('surprise',.6)
        for k in range(1,32):
            t=.6+k*.02;self.fresh(c,t,yaw=math.atan2(math.sin(k*.2),math.cos(k*.2)))
            c.emotion('surprise',t)
            self.assertGreater(c.tick(t)[1],0)
        self.fresh(c,1.3,yaw=math.atan2(math.sin(6.4),math.cos(6.4)))
        self.assertEqual(c.tick(1.3),(0.,0.))

    def test_loss_obstacle_and_sensor_timeout_latch_stop(self):
        for cause in ['face','front','rear','rotation','stale','jump']:
            c=self.ready();self.fresh(c,.6)
            c.emotion('angry' if cause=='rear' else 'surprise' if cause=='rotation' else 'fear',.6)
            c.tick(.6)
            if cause=='face':c.emotion('none',.7)
            if cause=='front':c.scan(.2,5,5,.7)
            if cause=='rear':c.scan(5,.2,5,.7)
            if cause=='rotation':c.scan(5,5,.2,.7)
            if cause=='jump':self.fresh(c,.7,x=2.)
            t=4 if cause=='stale' else .7
            self.assertEqual(c.tick(t),(0.,0.),cause)
            self.fresh(c,t+.1);c.emotion('fear',t+.1)
            self.assertEqual(c.tick(t+.1),(0.,0.),cause)

    def test_command_watchdog_even_with_fresh_sensors(self):
        c=self.ready();self.fresh(c,.6);c.emotion('fear',.6);c.tick(.6)
        self.fresh(c,2.2); self.assertEqual(c.tick(2.2),(0.,0.))

    def test_ready_latch_expires_if_detector_dies_while_idle(self):
        c=self.ready();self.assertTrue(c.armed)
        self.fresh(c,2.1);c.tick(2.1)
        self.assertFalse(c.armed)
        c.emotion('happy',2.2);self.assertEqual(c.tick(2.2),(0.,0.))

    def test_no_pose_or_invalid_scan_cannot_move(self):
        c=self.ready();c.scan(0,0,0,.6);c.emotion('fear',.6)
        self.assertEqual(c.tick(.6),(0.,0.))

    def test_progress_timeout_with_fresh_data(self):
        c=self.ready();self.fresh(c,.6);c.emotion('fear',.6);c.tick(.6)
        self.fresh(c,12);c.emotion('fear',12)
        self.assertEqual(c.tick(12),(0.,0.))

    def test_scan_inf_is_clear_nan_or_missing_sector_is_blocked(self):
        scan=[float('inf')]*360
        self.assertEqual(scan_clearances(scan,-math.pi,math.pi/180,.1,8),(8.,8.,8.))
        self.assertEqual(scan_clearances([float('nan')]*360,-math.pi,math.pi/180,.1,8),(0.,0.,0.))
        self.assertEqual(scan_clearances([1.]*10,0,.01,.1,8)[1],0.)
        # Seeing some front and rear rays is not enough for an entire rotation.
        self.assertEqual(scan_clearances([5.]*180,0,math.pi/180,.1,8)[2],0.)

    def test_rotated_lidar_front_and_rear_use_robot_frame(self):
        ranges=[5.]*360;ranges[90]=.2
        front,rear,around=scan_clearances(ranges,-math.pi,math.pi/180,.1,8,yaw_offset=math.pi/2)
        self.assertAlmostEqual(front,.2);self.assertEqual(rear,5.)
        ranges=[5.]*360;ranges[270]=.2
        self.assertAlmostEqual(scan_clearances(ranges,-math.pi,math.pi/180,.1,8,yaw_offset=math.pi/2)[1],.2)


if __name__=='__main__':unittest.main()
