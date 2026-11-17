from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/assign8_bringup/scripts'))
from camera_geometry import inference_frame, display_box


class GeometryTests(unittest.TestCase):
    def test_inference_is_bounded_without_upscaling(self):
        small,scale=inference_frame(np.zeros((480,640,3),np.uint8),320)
        self.assertEqual(small.shape,(240,320,3));self.assertEqual(scale,2.)
        tiny,scale=inference_frame(np.zeros((120,160,3),np.uint8),320)
        self.assertEqual(tiny.shape,(120,160,3));self.assertEqual(scale,1.)

    def test_face_box_is_scaled_back_to_display_frame(self):
        self.assertEqual(display_box({'x':10,'y':20,'w':30,'h':40},2.),(20,40,60,80))


if __name__=='__main__':unittest.main()
