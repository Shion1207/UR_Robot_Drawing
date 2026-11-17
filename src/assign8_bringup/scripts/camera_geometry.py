"""Bound AI input size while retaining the original webcam display resolution."""
import cv2


def inference_frame(frame,max_width=320):
    if max_width<=0:raise ValueError('inference width must be positive')
    height,width=frame.shape[:2]
    if width<=max_width:return frame,1.
    ratio=width/float(max_width)
    return cv2.resize(frame,(max_width,max(1,round(height/ratio))),interpolation=cv2.INTER_AREA),ratio


def display_box(region,scale):
    return tuple(round(float(region.get(key,0))*scale) for key in ('x','y','w','h'))
