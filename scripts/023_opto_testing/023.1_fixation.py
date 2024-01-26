#!/usr/bin/env python

import time
import os
import sys
import roslib
import rospy
import rostopic
import readline
import cPickle
import itertools
import yaml
import copy
import numpy as np
from std_msgs.msg import String
from exp_scripts.msg import MsgExpState
from exp_scripts.msg import MsgExpMetadata
from exp_scripts import git_tools
from ledpanels import display_ctrl
from muscle_imager.srv import SrvRefFrame
from muscle_imager.srv import SrvRefFrameRequest
from util.my_panel_lib import turn_off_panels, exc_visual_stim
from camera_strober import CameraStroberSerial

""" MAKE SURE THIS IS EXECUTABLE; USE 'chmod +x mypythonscript.py' in terminal """
############################################################################
########################### Metadata Information ###########################
############################################################################

exp_description = \
"""
Testing the combination of optogenetic activation and Ca2+ imaging

Here doing closed-loop stripe fixation. 

Going to test ATR-fed and ctrl flies in the following conditions:
    1) no lights 
    2) just blue light on
    3) blue and red light on

"""

fly_dob = '01.26.2024'

fly_genotype = """w[*] ; UAS-GCaMP6s/CyO ; R40D04-GAL4/+"""
genotype_nickname = 'C68'

head_fixed = False 
legs_cut = False
atr_food = False  # fed all trans retinal food ?

print genotype_nickname


############################################################################
########################### Script Variables ##########################
############################################################################

#Stimulus periods (in seconds)
STIM_DURATION = 3.0
PRESTIM_DURATION = 5.0
POSTSTIM_DURATION = 5.0

# conditions to test
CONDITIONS = ['none', 'blue', 'red']

# chrimson stim params
CHRIMSON_VOLTS = 5 
CHRIMSON_STR = 'set_a30 %s'  # NB: a30 is one of the entries in the Kinefly array 'a'

# how many trials per run
NUM_REPS = 5

# settings for 
#pattern playback rate 240 positions for 360deg
CL_GAIN_X = -2  # closed loop gain(?). was previously -2. alysha had it set up to -1; Francesca to 3

# string for visual stimulus
PATTERN_NAME = 'Pattern_bar.mat'

# name of unmixer being used (Thad's or Johan's)
UNMIXER_NAME = 'unmixer'  # 'unmixer' or 'live_viewer' 


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

# assert git_tools.check_git_status(repo_dirs)
git_SHA = git_tools.get_SHA_keys(repo_dirs)

# check if ROS is on yet
try:
    rostopic.get_topic_class('/rosout')
    is_rosmaster_running = True
except rostopic.ROSTopicIOException as err:
    is_rosmaster_running = False
    print(err)
    
#############################################################################
################################ Run experiment #############################
#############################################################################

if __name__ == '__main__':
    try:
        # ----------------------------------------------------------------------
        # create LED panel controller object
        print 'start'
        rospy.init_node('exp_script')
        exp_dir = script_dir
        ctrl = display_ctrl.LedControler()
        ctrl.load_SD_inf(exp_dir + '/firmware/panel_controller/SD.mat')
        
        # set up ROS publisher topics
        exp_pub = rospy.Publisher('/exp_scripts/exp_state', 
                                    String,
                                    queue_size = 10)
        meta_pub = rospy.Publisher('/exp_scripts/exp_metadata', 
                                    String,
                                    queue_size = 10)
        blk_pub = rospy.Publisher('/exp_scripts/exp_block',
                                    String,
                                    queue_size = 10)
        
        ch_pub = rospy.Publisher('/kinefly/flystate2phidgetsanalog/command',  # CHRIMSON
                                    String,
                                    queue_size = 10)
        
        # create instance of CameraStroberSerial
        css = CameraStroberSerial(port='/dev/triggerbox')
        
        # ----------------------------------------------------------------------
        # get left and right reference frames
        # rospy.wait_for_service('/unmixer_left/RefFrameServer')  # OUT OF DATE -- using live_viewer_(side) now
        # rospy.wait_for_service('/unmixer_right/RefFrameServer')
        
        time.sleep(1) # wait for all the publishers to come online
        
        # ----------------------------------------------------------------------
        # save metadata
        try:
            get_ref_frame_left = rospy.ServiceProxy('/%s_left/RefFrameServer'%(UNMIXER_NAME), SrvRefFrame) 
            print(get_ref_frame_left())
            rospy.logwarn(get_ref_frame_left())
        except (rospy.service.ServiceException, rospy.ROSException), e:
            print 'LEFT camera not in use: %s'%(e)
            rospy.logwarn('LEFT camera not in use: %s'%(e))
            get_ref_frame_left = lambda *args, **kwargs: None

        try:
            get_ref_frame_right = rospy.ServiceProxy('/%s_right/RefFrameServer'%(UNMIXER_NAME), SrvRefFrame) 
            print(get_ref_frame_right())
            rospy.logwarn(get_ref_frame_right()) 
        except (rospy.service.ServiceException, rospy.ROSException), e:
            print 'RIGHT camera not in use: %s'%(e)
            rospy.logwarn('RIGHT camera not in use: %s'%(e))
            get_ref_frame_right = lambda *args, **kwargs: None

        metadata =   {'git_SHA':git_SHA,
                      'script_path':script_path,
                      'exp_description':exp_description,
                      'script_code':script_code,
                      'fly_dob':fly_dob,
                      'fly_genotype':fly_genotype,
                      'genotype_nickname':genotype_nickname,
                      'head_fixed':head_fixed,
                      'legs_cut':legs_cut,
                      'atr_food':atr_food}

        meta_pub.publish(cPickle.dumps(metadata))
        
        
        ###################################################################################
        # Run experiment
        ###################################################################################

        # get start time
        t0 = time.time()
        
        # stop panels 
        ctrl.stop()
        
        # start closed-loop stripe movement -- going to run this throughout
        ctrl.set_position_function_by_name('X', 'default')  # not sure what this does
        ctrl.set_pattern_by_name(PATTERN_NAME)  # set pattern 
        ctrl.set_position(0, 0)  # set initial position
        ctrl.set_mode('xrate=ch0','yrate=funcy')  # not really sure what this does
        ctrl.send_gain_bias(gain_x=CL_GAIN_X, gain_y=0, bias_x=0, bias_y=0) # set gain and bias for panel
    
        # execute panel motion
        ctrl.start()
        
        # publish the state
        exp_pub.publish('closed_loop;gain=%s'%(CL_GAIN_X))
        blk_pub.publish('stripe_fix')
        
        # pause a bit before printing commands
        time.sleep(5)

        # loop over repetitions
        for rep in range(NUM_REPS):
            # loop over condition types
            for condition in np.random.permutation(CONDITIONS):
                rospy.logwarn('enter light condition: %s'%(condition))
                
                # ------------------------------------------------
                # pre-stimulus closed loop stripe fixation
                exp_pub.publish('closed_loop;visual;light=%s;prestim'%(condition))
                time.sleep(PRESTIM_DURATION)
                
                # ------------------------------------------------
                # stimulus period
                exp_pub.publish('closed_loop;visual;light=%s;stim'%(condition))
                
                # set red light voltage (0 if not doing chrimson this condition)
                if condition == 'red':
                    channel_volts = CHRIMSON_VOLTS
                else:
                    channel_volts = 0
                    
                ch_pub.publish(CHRIMSON_STR%(channel_volts))
                    
                # set blue light options (camera_strober)
                if condition == 'blue':
                    css.disable_led()
                    
                # sleep
                time.sleep(STIM_DURATION)
                
                # undo any changes to red or blue lights
                ch_pub.publish(CHRIMSON_STR%(0))
                css.enable_led()
                
                # ------------------------------------------------
                # post-stimulus closed loop stripe fixation
                exp_pub.publish('closed_loop;visual;light=%s;poststim'%(condition))
                time.sleep(POSTSTIM_DURATION)
        

        #################################################
        # Wind down expt
        #################################################

        #publish a refrence frame as a status message to mark the end of the experiment.
        print(get_ref_frame_left())
        print(get_ref_frame_right())

        meta_pub.publish(cPickle.dumps(metadata))
        
        # print some stuff at the end to let us know we're done!
        rospy.logwarn('end_of_experiment')
        rospy.logwarn(time.time()-t0)
        
        # turn off panels
        turn_off_panels(ctrl)

    except rospy.ROSInterruptException:
        print ('exception')
        pass
