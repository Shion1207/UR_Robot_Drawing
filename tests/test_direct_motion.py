import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/assign8_bringup/scripts'))
from emotion_logic import DirectMotionController


class DirectTests(unittest.TestCase):
    def fresh(self,c,t,x=0,yaw=0):
        c.pose(x,0,yaw,t);c.scan(5,5,5,t)

    def test_smile_starts_without_neutral_and_sad_switches_directly(self):
        c=DirectMotionController();self.fresh(c,0);c.emotion('happy',0)
        self.assertGreater(c.tick(0)[1],0)
        self.fresh(c,.1,.4);c.emotion('happy',.1)
        self.assertGreater(c.tick(.1)[1],0)
        c.emotion('sad',.2);self.assertLess(c.tick(.2)[1],0)
        c.emotion('neutral',.3);self.assertEqual(c.tick(.3),(0.,0.))

    def test_step_and_spin_finish_once_until_expression_changes(self):
        for label in ('fear','angry','surprise'):
            c=DirectMotionController();self.fresh(c,0);c.emotion(label,0);c.tick(0)
            if label=='surprise':
                for k in range(1,34):
                    self.fresh(c,k*.02,yaw=math.atan2(math.sin(k*.2),math.cos(k*.2)))
                    c.emotion(label,k*.02);c.tick(k*.02)
            else:self.fresh(c,.2,.16);c.tick(.2)
            c.emotion(label,.8);self.assertEqual(c.tick(.8),(0.,0.))
            c.emotion('neutral',.9);c.emotion(label,1.)
            self.assertNotEqual(c.tick(1.),(0.,0.))

    def test_missing_face_and_obstacle_still_stop(self):
        c=DirectMotionController();self.fresh(c,0);c.emotion('happy',0);c.tick(0)
        c.emotion('none',.1);self.assertEqual(c.tick(.1),(0.,0.))
        c.emotion('sad',.2);c.scan(.1,.1,.1,.2)
        self.assertEqual(c.tick(.2),(0.,0.))


if __name__=='__main__':unittest.main()
