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
from exp_scripts.msg import MsgExpState
from exp_scripts.msg import MsgExpMetadata
# from exp_scripts import git_tools
from ledpanels import display_ctrl
# from muscle_imager.srv import SrvRefFrame
# from muscle_imager.srv import SrvRefFrameRequest

############################################################################
########################### Script Variables ###############################
############################################################################


#Stimulus periods
MOTION_DURATION = 10.0
PAUSE_DURATION = 3.0 

#pattern playback rate 240 positions for 360deg
PLAYBACK_LEVEL = 24 # 60*2 #Hz = 90deg/sec
CL_GAIN_X = -1

#construct the list of motion patterns we will test. Three different
#patterns for each type of motion.
# PATTERN_LIST = ['cl_stripe']  # ;g_x=%s
PATTERN_LIST = ['ol_pitch_%s'%(d) for d in ['down','up']]
PATTERN_LIST.extend(['ol_roll_%s'%(d) for d in ['left','right']])
PATTERN_LIST.extend(['ol_yaw_%s'%(d) for d in ['left','right']])

# path to current script
script_path = os.path.realpath(sys.argv[0])
script_dir = os.path.dirname(script_path)

###########################################################################
#################### HELPER FUNCTIONS #####################################
###########################################################################
"""
Quick function to define a dictionary that takes pattern strings as inputs and outputs SD card filenames
...could be improved a lot (include gain/bias params?)

"""
def get_pattern_dict():
    pattern_dict = {'stripe'     : 'Pattern_bar.mat',
                    'yaw_right'  : 'Pattern_rot_axis_5.mat',
                    'yaw_left'   : 'Pattern_rot_axis_4.mat',
                    'pitch_up'   : 'Pattern_rot_axis_0.mat',
                    'pitch_down' : 'Pattern_rot_axis_2.mat',
                    'roll_left'  : 'Pattern_rot_axis_3.mat',
                    'roll_right' : 'Pattern_rot_axis_1.mat',
                    }

    return pattern_dict

# ------------------------------------------

"""
Turn panels ON for a fixed duration

 blk_pub.publish('all_on')
"""
def turn_on_panels(duration=1):

    ctrl.stop()
    ctrl.all_on()
    time.sleep(duration)
    ctrl.stop()

# ------------------------------------------

"""
Turn panels OFF for a fixed duration

 blk_pub.publish('all_off')

"""
def turn_off_panels(duration=1):

    ctrl.stop()
    ctrl.all_off()
    time.sleep(duration)
    ctrl.stop()

# ------------------------------------------

"""
Function to execute generic visual stimulus

INPUTS:
    - ctrl: LED panel display controller object
    - block_name: string describing the type of visual stimulus. should be of the form '(feedback type)_(rot axis)_(direction)'
        i.e. 'ol_yaw_left' = open loop yaw left or 'cl_stripe' = closed loop stripe
    - duration: time to show stimulus in seconds
    - gain_x, gain_y: gains in x, y directions
    - bias_x, bias_y: bias in x, y directions
    - ch: boolean for using a Chrimson stimulus or not (under construction!)
"""

def exc_visual_stim(ctrl, block_name, duration, gain_x=0, gain_y=0, bias_x=0, bias_y=0, ch=0):
    
    # print stimulus name so we can see it
    print block_name

    # try to extract some info from the string pattern
    block_name_split = block_name.split('_')

    # first see if we're doing open or closed loop 
    if (len(block_name_split)<3) or (block_name_split[0]=='cl'):
        open_loop_flag = False
        xrate_fun = 'ch0'  # still not *sure* what this does, but i think it makes the x rate dependent on analog in
        y_idx = 0  # index in the "y" position (4th dim of pattern array)
        x_init = 24  # start value "x" position of pattern (3rd dim of pattery array) 

    elif block_name_split[0]=='ol':
        open_loop_flag = True
        xrate_fun = 'funcx'
        y_idx = 1  # index in the "y" position (4th dim of pattern array)
        x_init = np.random.randint(0,96)  # start value "x" position of pattern (3rd dim of pattery array) 

    else:
        raise Exception("Could not determine feedback type (open vs closed loop)")

    # next determine rotational axis/visual pattern. 
    # NB: this determines what to load off of SD card
    pat_str = '_'.join(block_name_split[1:])
    pattern_dict = get_pattern_dict()
    pattern_name = pattern_dict[pat_str]

    # intialize led panels
    ctrl.stop()
    ctrl.set_position_function_by_name('X', 'default')  # not sure what this does
    ctrl.set_pattern_by_name(pattern_name)  # set pattern 
    ctrl.set_position(x_init, y_idx)  # set initial position
    ctrl.set_mode('xrate=%s'%(xrate_fun),'yrate=funcy')  # not really sure what this does
    ctrl.send_gain_bias(gain_x=gain_x, gain_y=gain_y, bias_x=bias_x, bias_y=bias_y) # set gain and bias for panel
    
    # execute panel motion
    ctrl.start()
    time.sleep(duration)
    ctrl.stop()

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

    # read out some vars
    motion_duration = MOTION_DURATION
    pause_duration = PAUSE_DURATION
    
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
        exp_msg.state = 'pattern=%s'%(cond)
        exp_pub.publish(exp_msg)

        # execute visual pattern
        exc_visual_stim(ctrl, cond, motion_duration, gain_x=PLAYBACK_LEVEL, gain_y=0, bias_x=0, bias_y=0)
        
        # pause for a bit in between stimuli
        time.sleep(pause_duration)
    
    # --------------------------------------------------------------------------
    # end with some more closed loop fixation
    exc_visual_stim(ctrl, cond_cl, motion_duration, gain_x = CL_GAIN_X)
    time.sleep(pause_duration)
    
    print 'over'
    
    blk_pub.publish('all_off')
    turn_off_panels()    


