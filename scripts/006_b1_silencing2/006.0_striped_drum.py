#!/usr/bin/env python

import time
import os
import sys
import roslib
import rospy
from std_msgs.msg import String
from exp_scripts.msg import MsgExpState
from exp_scripts.msg import MsgExpMetadata
from exp_scripts import git_tools
from ledpanels import display_ctrl
from muscle_imager.srv import SrvRefFrame
from muscle_imager.srv import SrvRefFrameRequest

############################################################################
########################### Metadata Information ###########################
############################################################################

exp_description = \
""" Determination of effect of B1 silencing on flight motor codes ---

The line is:

Legs cut off. Not head fixed.

epi_level = 0.125

"""
fly_dob = '4.30.2017'
fly_genotype = """w/w ; GMR39E01-LexA/GCaMP6f-LexOp ; +/UAS-kir2.1::gfp""" #UAS-kir2.1::gfp
genotype_nickname = "E-kir/+"

############################################################################
########################### Script Variables ###############################
############################################################################

EPI_LEVEL = 0.125 # Voltage sent to Blue LED for imaging GCaMP
#Stimulus periods
MOTION_DURATION = 3.0
PREMOTON_DURATION = 5.0
POSTMOTION_DURATION = 5.0
FIXATION_DURATION = 5.0
#pattern playback rate 240 positions for 360deg
PLAYBACK_LEVEL = 50 #Hz = 90deg/sec
REP_NUMBER = 5 
#construct the list of motion patterns we will test. Three different
#patterns for each type of motion.

############################################################################
########################### Initialize Experiment ##########################
############################################################################

script_path = os.path.realpath(sys.argv[0])
script_dir = os.path.dirname(script_path)
#load the script to publish as message
with open(script_path,'rt') as f:
    script_code = f.read() 
#list of all git tracked repositories
with open(os.path.join(script_dir,'tracked_git_repos.txt')) as f:
    repo_dirs = f.readlines() 
assert git_tools.check_git_status(repo_dirs)
git_SHA = git_tools.get_SHA_keys(repo_dirs)

#############################################################################
################################ Run experiment #############################
#############################################################################

if __name__ == '__main__':
    try:
        import numpy as np
        print 'start'
        rospy.init_node('exp_script')
        exp_dir = script_dir
        ctrl = display_ctrl.LedControler()
        ctrl.load_SD_inf(exp_dir + '/firmware/SD.mat')
        
        exp_pub = rospy.Publisher('/exp_scripts/exp_state', 
                                    MsgExpState,
                                    queue_size = 10)
        exp_msg = MsgExpState()
        meta_pub = rospy.Publisher('/exp_scripts/exp_metadata', 
                                    MsgExpMetadata,
                                    queue_size = 10)
        
        rospy.wait_for_service('RefFrameServer')
        get_ref_frame = rospy.ServiceProxy('RefFrameServer', SrvRefFrame)
        # init experiment
        time.sleep(5) # wait for all the publishers to come online

        ############################################################
        #call get_ref_frame service, this will not only get the 
        #user-defined reference frame but also publish the frame
        #as a message that can be logged to a bag file.
        ############################################################
        print(get_ref_frame())
        meta_pub.publish(git_SHA = git_SHA,
                         script_path = script_path,
                         exp_description = exp_description,
                         script_code = script_code,
                         fly_dob = fly_dob,
                         fly_genotype = fly_genotype,
                         genotype_nickname = genotype_nickname)
        
        ## set the imaging light level
        ctrl.set_ao(4,EPI_LEVEL)

        conditions = ['Left','Right']

        #Run experiment
        for rep in range(REP_NUMBER):
            print rep
            for condition in np.random.permutation(conditions):


                print condition
                #################################################
                # Closed Loop
                #################################################
                print 'enter closed loop stripe fixation'
                ctrl.stop()
                ctrl.set_pattern_by_name('Pattern_fixation_4_wide_4X12_Pan.mat')
                ctrl.set_position(0,0)
                #ctrl.set_function_by_name('X','default',freq=60)
                ctrl.send_gain_bias(gain_x = -70,bias_x = 0.0)
                ctrl.set_mode('xrate=ch0','yrate=funcy')
                ctrl.start()
                ### publish the state
                exp_msg.state = 'closed_loop;gain=-5'
                exp_pub.publish(exp_msg)
                time.sleep(FIXATION_DURATION)
        
                #################################################
                # Open Loop
                #################################################

                ctrl.stop()
                ctrl.set_pattern_by_name('Pattern_multi_width_optomotor_patterns_48_Pan.mat')
                ctrl.set_position(0,0)
                exp_msg.state = 'open_loop;visual;pattern=%s;static'%(condition)
                exp_pub.publish(exp_msg)
                time.sleep(PREMOTON_DURATION)
                #ctrl.set_function_by_name('X','default',freq=60)
                if condition == 'Left':
                    ctrl.send_gain_bias(bias_x = 0,gain_x = PLAYBACK_LEVEL*-1,gain_y = 0)
                if condition == 'Right':
                    ctrl.send_gain_bias(bias_x = 0,gain_x = PLAYBACK_LEVEL,gain_y = 0)
                ctrl.set_mode('xrate=funcx','yrate=funcy')
                ctrl.start()
                exp_msg.state = 'open_loop;visual;pattern=%s;motion'%(condition)
                exp_pub.publish(exp_msg)
                time.sleep(MOTION_DURATION)
                ctrl.stop()
                exp_msg.state = 'open_loop;visual;pattern=%s;static'%(condition)
                exp_pub.publish(exp_msg)
                time.sleep(POSTMOTION_DURATION)

        #publish a refrence frame as a status message to mark the end of the experiment.
        get_ref_frame()
        meta_pub.publish(git_SHA = git_SHA,
                         script_path = script_path,
                         exp_description = exp_description,
                         script_code = script_code)

        #################################################
        # Closed Loop
        #################################################
        print 'enter closed loop stripe fixation'
        ctrl.stop()
        ctrl.set_pattern_by_name('Pattern_fixation_4_wide_4X12_Pan.mat')
        ctrl.set_position(0,0)
        ctrl.set_function_by_name('Y','default',freq=50)
        ctrl.send_gain_bias(gain_x = -70,bias_x = 0.0)
        ctrl.set_mode('xrate=ch0','yrate=funcy')
        ctrl.start()
        ### publish the state
        exp_msg.state = 'closed_loop;gain=-5'
        exp_pub.publish(exp_msg)
        time.sleep(1)

    except rospy.ROSInterruptException:
        print 'exception'
        pass