"""
Quick script to initialize camera status, which can be executed at roslaunch. 

Need to handle three cases: imaging from both sides, imaging from the right, and imaging from the left
"""
import rospy
from camera_strober_serial import CameraStroberSerial

# ------------------------------------------------------------------
# Read in the case that we're using ('both' | 'right' | 'left')
# ------------------------------------------------------------------
NODE_NAME = 'camera_serial'
ARG_NAME = 'case'
cam_case = rospy.get_param('/%s/%s'%(NODE_NAME, ARG_NAME)) # node_name/argsname

# -----------------------------------------
# Execute code to enable/disable cameras
# -----------------------------------------
if __name__ == '__main__':

    # create instance of CameraStroberSerial
    css = CameraStroberSerial()
    
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
        css.enable_cam("right")
        css.disable_cam("left")
        
    else:
        # this means we got a bad case input...
        raise ValueError, "Camera case must be one of: both, right, or left"
    
    # also use this as an opportunity to test the "get" function
    cam_right_enabled = css.get_cam_state("right")
    cam_left_enabled = css.get_cam_state("left")
    
    # print('Right camera state: %d'%(cam_right_enabled))
    # print('Left camera state: %d'%(cam_left_enabled))
    rospy.logwarn('Right camera state: %d'%(cam_right_enabled))
    rospy.logwarn('Left camera state: %d'%(cam_left_enabled))
    
