#!/usr/bin/env python

import time
import os
import sys

import roslib
#roslib.load_manifest('ledpanels')
import rospy

from std_msgs.msg import String
from exp_scripts.msg import MsgExpState
from exp_scripts.msg import MsgExpMetadata

from exp_scripts import git_tools,display_ctrl

from muscle_imager.srv import SrvRefFrame
from muscle_imager.srv import SrvRefFrameRequest

#####################################################################################
########################### Initialize Experiment ###################################
#####################################################################################
exp_description = \
    """Testing the effect of the imaging light intensity on the chrimson response. 
       The line is S-28 X C-85.
       Chrimson is expressed using SS01580 in DN106."""

#list of all git tracked repositories
#repo_root = '/home/imager/catkin/src'
script_path = os.path.realpath(sys.argv[0])
script_dir = os.path.dirname(script_path)
with open(script_path,'rt') as f:
    script_code = f.read()
with open(os.path.join(script_dir,'tracked_git_repos.txt')) as f:
	repo_dirs = f.readlines() 
assert git_tools.check_git_status(repo_dirs)
git_SHA = git_tools.get_SHA_keys(repo_dirs)
fly_dob = '3.25.2017'
fly_genotype = 'S-28 X C-85'

#####################################################################################
#####################################################################################

if __name__ == '__main__':
    try:
        import numpy as np
        print 'start'
        rospy.init_node('exp_script')
        exp_dir = script_dir
        ctrl = display_ctrl.LedControler()
        ctrl.load_SD_inf(exp_dir + '/SD.mat')

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
        time.sleep(5)

        #call get_ref_frame service, this will not only publish the
        #user defined reference frame but also publish a refrence
        #frame message
        print(get_ref_frame())
        meta_pub.publish(git_SHA = git_SHA,
                         script_path = script_path,
                         exp_description = exp_description,
                         script_code = script_code,
                         fly_dob = fly_dob,
                         fly_genotype = fly_genotype)

        for rep in range(10):
            #for vlevel in np.random.permutation(range(1000,16001,5000)):
            for vlevel in np.random.permutation([400,800,1200,1600]):
                print vlevel
                ctrl.stop()
                #################################################
                # Closed Loop
                #################################################
                print 'enter closed loop stripe fixation'
                ctrl.set_pattern_by_name('Pattern_fixation_4_wide_4X12_Pan.mat')
                ctrl.set_position(0,0)
                ctrl.set_function_by_name('Y','default',freq=50)
                ctrl.send_gain_bias(gain_x = -90,bias_x = 0.0)
                ctrl.set_mode('xrate=ch0','yrate=funcy')
                ### set the imaging level       
                ctrl.set_ao(4,vlevel)
                ctrl.start()
                ### publish the state
                exp_msg.state = 'closed_loop;gain=-5;epi_level=%s'%(vlevel)
                exp_pub.publish(exp_msg)
                time.sleep(5)
        
                #################################################
                # Open Loop
                #################################################
                print 'all panels off'
                ctrl.stop()
                ctrl.all_off()
                exp_msg.state = 'all_off;epi_level=%s'%(vlevel)                
                exp_pub.publish(exp_msg)
                time.sleep(5.0)
                ### publish the state
                exp_msg.state = 'led_pulse;epi_level=%s'%(vlevel)               
                exp_pub.publish(exp_msg)
                print '617nm pulse'
                ctrl.set_ao(3,15000)
                time.sleep(0.5)
                ctrl.set_ao(3,0)
                time.sleep(5.0)
                print rep
                ctrl.set_position(0,20)
                ctrl.stop()

        #publish a refrence frame an status message to mark the end of the experiment.
        get_ref_frame()
    	meta_pub.publish(git_SHA = git_SHA,
                         script_path = script_path,
                         exp_description = exp_description,
                         script_code = script_code)

    except rospy.ROSInterruptException:
        print 'exception'
        pass