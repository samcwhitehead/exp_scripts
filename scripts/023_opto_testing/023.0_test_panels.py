# !/usr/bin/env python
###########################################################################
# work space for making a more efficient visual stim function
# ...maybe convert into a test of the visual panel?
###########################################################################
import time
import os
import sys
import roslib
import rospy
import numpy as np
from std_msgs.msg import String
from ledpanels import display_ctrl
from util.my_panel_lib import turn_off_panels, exc_visual_stim


############################################################################
########################### Script Variables ###############################
############################################################################

#pattern playback rate 240 positions for 360deg
PLAYBACK_LEVEL = 60 # 60*2 #Hz = 90deg/sec
CL_GAIN_X = -1

#Stimulus periods
MOTION_DURATION = 10  # 96/(1.4*PLAYBACK_LEVEL)
PAUSE_DURATION = 5.0 

# construct the list of motion patterns we will test
PATTERN_LIST = ['ol_pitch_%s'%(d) for d in ['down','up']]
PATTERN_LIST.extend(['ol_roll_%s'%(d) for d in ['left','right']])
PATTERN_LIST.extend(['ol_yaw_%s'%(d) for d in ['left','right']])

# path to current script
script_path = os.path.realpath(sys.argv[0])
script_dir = os.path.dirname(script_path)

###########################################################################
#################### MAIN  ################################################
###########################################################################

if __name__ == '__main__':
    # create LED panel controller object
    print 'start'
    rospy.init_node('exp_script')
    exp_dir = script_dir
    ctrl = display_ctrl.LedControler()
    ctrl.load_SD_inf(exp_dir + '/firmware/panel_controller/SD.mat')
    
    # set up ROS publishers
    exp_pub = rospy.Publisher('/exp_scripts/exp_state', 
                                String,
                                queue_size = 10)

    blk_pub = rospy.Publisher('/exp_scripts/exp_block',
                                String,
                                queue_size = 10)
    
    ch_pub = rospy.Publisher('/kinefly/flystate2phidgetsanalog/command',  # CHRIMSON
                                    String,
                                    queue_size = 10)

    # read out some vars
    motion_duration = MOTION_DURATION
    pause_duration = PAUSE_DURATION
    
    # read out start time
    t0 = time.time()
    
    # --------------------------------------------------------------------------
    # start with some closed loop stripe fixation
    cond_cl = 'cl_stripe'
    exc_visual_stim(ctrl, cond_cl, motion_duration, gain_x = CL_GAIN_X)
    time.sleep(pause_duration)
    
    # --------------------------------------------------------------------------
    # loop over visual patterns and execute them
    condition_list = PATTERN_LIST

    for cond in condition_list:
        # publish the type of stimulus to the blk ros channel
        blk_pub.publish(cond)
        
        # also publish to exp topic
        # exp_msg.state = pattern=%s'%(cond)
        exp_pub.publish('pattern=%s'%(cond))
        print cond
        
        # execute visual pattern
        exc_visual_stim(ctrl, cond, motion_duration, gain_x=PLAYBACK_LEVEL, gain_y=0, bias_x=0, bias_y=0)
        
        # pause for a bit in between stimuli
        time.sleep(pause_duration)
        
        # try a light stimulus
        print 'test light pulse'
        ch_pub.publish('set_a30 %s'%(1))
        time.sleep(2)
        ch_pub.publish('set_a30 0')
        time.sleep(2)
        
    
    # --------------------------------------------------------------------------
    # end with some more closed loop fixation
    exc_visual_stim(ctrl, cond_cl, motion_duration, gain_x = CL_GAIN_X)
    time.sleep(pause_duration)
    
    print 'end of experiment'
    print (time.time()-t0)
    
    blk_pub.publish('all_off')
    turn_off_panels(ctrl)    


