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
fly_dob = 'x.x.x'
fly_genotype = """Formal Genotype"""
genotype_nickname = "Lab Nickname"

############################################################################
########################### Script Variables ###############################
############################################################################

EPI_LEVEL = 0.125 # Voltage sent to Blue LED for imaging GCaMP
TRIAL_DURATION = 10

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
                        print 'enter closed loop stripe fixation'

        ctrl.stop()
        ctrl.set_pattern_by_name('Pattern_fixation_4_wide_4X12_Pan.mat')
        ctrl.set_position(0,0)
        ctrl.set_function_by_name('X','default',freq=60)
        ctrl.send_gain_bias(gain_x = -90,bias_x = 0.0)
        ctrl.set_mode('xrate=ch0','yrate=funcy')
        ctrl.start()
        ### publish the state
        exp_msg.state = 'closed_loop;gain=-5'
        exp_pub.publish(exp_msg)
        time.sleep(5.0)

        ## set the imaging light level
        ctrl.set_ao(4,EPI_LEVEL)
        #Run experiment
        for rep in range(2):
            print('puff')
            time.sleep(15)
            print('stop')
            time.sleep(15)

        #publish a refrence frame as a status message to mark the end of the experiment.
        get_ref_frame()
        meta_pub.publish(git_SHA = git_SHA,
                         script_path = script_path,
                         exp_description = exp_description,
                         script_code = script_code)
        
    except rospy.ROSInterruptException:
        print 'exception'
        pass