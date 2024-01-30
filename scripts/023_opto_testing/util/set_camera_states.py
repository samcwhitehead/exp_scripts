#!/usr/bin/env python
"""
Quick script to initialize camera status, which can be executed at roslaunch. 

Need to handle three cases: imaging from both sides, imaging from the right, and imaging from the left
"""
# import os
import socket
import rospy
from camera_strober import CameraStroberSerial

import camera_strober
import os
print(os.path.abspath(camera_strober.__file__))
# -----------------------------------------------------------------------------
# Read in the case that we're using ('both' | 'right' | 'left') 
# -----------------------------------------------------------------------------
# NB: if there's no ros param to read, just set the cameras to "both"
try:
    cam_case = rospy.get_param('/cam_case')
except (KeyError, socket.error):
    cam_case = "both"

# rospy.logwarn('Input camera case: %s'%(cam_case))
print('Input camera case: %s'%(cam_case))

## -----------------------------------------
## Execute code to enable/disable cameras
## -----------------------------------------
if __name__ == '__main__':

    # create instance of CameraStroberSerial
    css = CameraStroberSerial(port='/dev/triggerbox')
  
    # based on case, set cameras as enabled/disabled
    if cam_case == "both":
        # in this case, both cameras should be on
        css.enable_cam("right")
        css.enable_cam("left")
        
    elif cam_case == "right":
        # in this case, right camera should be on and left camera off
        css.enable_cam("right")
        css.disable_cam("left")
        
    elif cam_case == "left":
        # in this case, right camera should be off and left camera on
        css.enable_cam("left")
        css.disable_cam("right")
        
    else:
        # this means we got a bad case input...
        raise ValueError("Camera case must be one of: both, right, or left")
    
    # also use this as an opportunity to test the "get" function
    cam_right_enabled = css.get_cam_state("right")
    cam_left_enabled = css.get_cam_state("left")
    
    print('Right camera state: %d'%(cam_right_enabled))
    print('Left camera state: %d'%(cam_left_enabled))
    # rospy.logwarn('Right camera state: %d'%(cam_right_enabled))
    # rospy.logwarn('Left camera state: %d'%(cam_left_enabled))
    
    # test LED off/on
    css.disable_led()
    css.enable_led()
