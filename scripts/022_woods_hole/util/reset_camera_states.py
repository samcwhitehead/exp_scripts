#!/usr/bin/env python
"""
Quick script to reset camera status, which can be run anywhere(?)

Always enable both cameras
"""
from camera_strober import CameraStroberSerial


## -----------------------------------------
## Execute code to enable both cameras
## -----------------------------------------
if __name__ == '__main__':

    # create instance of CameraStroberSerial
    css = CameraStroberSerial()
   
    # enable both cameras
    css.enable_cam("right")
    css.enable_cam("left")
    
    # check that this worked
    cam_right_enabled = css.get_cam_state("right")
    cam_left_enabled = css.get_cam_state("left")
    
    print('Right camera state: %d'%(cam_right_enabled))
    print('Left camera state: %d'%(cam_left_enabled))
   
