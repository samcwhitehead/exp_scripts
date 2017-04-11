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
"""Modification of previous experiment -- 
Changes that were made:
    * Reduced rep numbers from 10 to 5
    * Reduce epi illumitation ctrl voltage from 0.15V to 0.125V 
    * Added two conditions to led pulse and visual expansion
    * Will collect a period of quiescence and fiber targeting (with IR light)
    * Changed the encoding string in script to make the data easier to analyze.

Summary from previous experiment script:
Comparison the motor program elicited by DN106 stimulation vs visual stimlus:
I will combine an optogenetic stimlus and a visual stimlus in the same set of
flies and ask do these stimuli recrut the same set of flight muscles? The 
protocol will explore a range of chrimson activation powers and stimulus
speeds. After considering the possibility, I will NOT include a trial where 
expansion and chrimson are presented together. The reasoning here is that 
studying the interaction of the two systems will be more informative if we 
first understand the input-output and recrutment sequence of the programs 
independently. For the visual series, I will use the same sequence of stimuli
that I used in 000_sys_test. For the led powers I will titrate over a range
of 4 intensity levels but use a shorter (100ms) pulse. This is because when
imaging using the high camera gain I am required to use in order to minimize  
activating chrimson there is considerable bleedthrough of the 617nm light in 
the imaging camera. The short duration will at least minimize the time over 
which this artifact is an issue.

The voltages used to control the epi-led in the pilot experiments were:
[ 0.122,  0.244,  0.366,  0.488]. Somewhere between 0.12 and 0.24 seemed 
to be the best, so I will go with 0.15. to power the blue light. I run the
chrimson light at 0.5, 1 , 3,and 5V.

The line is S-28 X C-85. Flies are rased on retinal bottels made on 4.7
with 1:250 dilution (200uL) of 100mm ATR.

Legs cut off. Not head fixed.

Chrimson is expressed using SS01580 in DN106.

pulse_voltages = [ 0.1, 0.5, 1.0, 2.0, 3.0, 5.0]
epi_level = 0.125
"""

fly_dob = '4.7.2017'
genotype_nickname = "S-28 X C-85 39E01-GcAMP,DN106-Chrimson"
fly_genotype = """ w+; 
P{y[+t7.7] w[+mC]=13XLexAop2-IVS-GCaMP6f-p10}su(Hw)attP5, P{y[+t7.7] w[+mC]=GMR38H06-lexA}attP40 / P{w[+mC]=BJD115F05-p65ADzpUw}attP40;
P{20XUAS-IVS-CsChrimson.mVenus}attP2 / P{w[+mC]=GMR48E11-ZpGal4DBDUw}attP2"""
pulse_voltages = [ 0.1, 0.5, 1.0, 2.0, 3.0, 5.0]
epi_level = 0.125

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
        #call get_ref_frame service, this will not get the current
        #user-defined reference frame but also publish the refrence
        #frame as a message
        ############################################################
        print(get_ref_frame())
        meta_pub.publish(git_SHA = git_SHA,
                         script_path = script_path,
                         exp_description = exp_description,
                         script_code = script_code,
                         fly_dob = fly_dob,
                         fly_genotype = fly_genotype,
                         genotype_nickname = genotype_nickname)

        #Set up list of conditions
        conditions = [('visual',param) for param in ctrl.funcstrings]
        conditions.extend([('opto',param) for param in pulse_voltages])
        ## set the imaging light level
        ctrl.set_ao(4,epi_level)

        for c in conditions:
        	print c
        #Run experiment
        for rep in range(5):
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
                ctrl.set_function_by_name('Y','default',freq=50)
                ctrl.send_gain_bias(gain_x = -90,bias_x = 0.0)
                ctrl.set_mode('xrate=ch0','yrate=funcy')
                ctrl.start()
                ### publish the state
                exp_msg.state = 'closed_loop;gain=-5'
                exp_pub.publish(exp_msg)
                time.sleep(5)
        
                #################################################
                # Open Loop
                #################################################
                if condition[0] == 'opto':
                    print 'all panels on'
                    ctrl.stop()
                    ctrl.all_on()
                    exp_msg.state = 'open_loop;opto;power_level=%s;pulse_off'%str(condition[1])               
                    exp_pub.publish(exp_msg)
                    time.sleep(2.0)
                    ### publish the state
                    exp_msg.state = 'open_loop;opto;power_level=%s;pulse_on'%str(condition[1])
                    exp_pub.publish(exp_msg)
                    ctrl.set_ao(3,float(condition[1]))
                    time.sleep(0.1)
                    exp_msg.state = 'open_loop;opto;power_level=%s;pulse_off'%str(condition[1])
                    exp_pub.publish(exp_msg)
                    ctrl.set_ao(3,0)
                    time.sleep(3.0)
                    ctrl.set_position(0,20)
                if condition[0] == 'visual':
                    ctrl.stop()
                    ctrl.set_pattern_by_name('Pattern_linear_expansion_48_Pan.mat')
                    ctrl.set_position(0,20)
                    ctrl.set_function_by_name('X',condition[1],freq=200)
                    ctrl.set_function_by_name('Y','default',freq=200)
                    ctrl.send_gain_bias(gain_x = 1,gain_y = 0)
                    ctrl.set_mode('x=x0+funcx','yrate=funcy')
                    ctrl.start()
                    ### publish the state
                    exp_msg.state = 'open_loop;visual;vfunc=%s'%str(condition[1])
                    exp_pub.publish(exp_msg)
                    time.sleep(5.0)

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
        ctrl.send_gain_bias(gain_x = -90,bias_x = 0.0)
        ctrl.set_mode('xrate=ch0','yrate=funcy')
        ctrl.start()
        ### publish the state
        exp_msg.state = 'closed_loop;gain=-5'
        exp_pub.publish(exp_msg)
        time.sleep(1)

    except rospy.ROSInterruptException:
        print 'exception'
        pass