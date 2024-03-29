#!/usr/bin/env python

import time
import os
import sys
import roslib
import rospy
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

############################################################################
########################### Metadata Information ###########################
############################################################################

exp_description = \
""" Simultaneous imaging of muscle activity and tendon movement ---

Testing to see if I can image both calcium activity in muscles AND tendon movement

NB: While happending in the same fly, the muscles and tendons are being imaged on separate sides

Legs cut off; head NOT fixed (for now)

In this particular experiment paradigm, looking at response to visual stimuli

"""

fly_dob = '07.07.2023'

fly_genotype = """w[*] ; +/+ ; 20XUAS-jGCaMP7f/R39E01-GAL4"""
genotype_nickname = '39E01'
#fly_genotype = """w[*] ; +/+ ; 20XUAS-jGCaMP7f/R22H05-GAL4"""
#genotype_nickname = '15D'


head_fixed = False 
legs_cut = False

print genotype_nickname

############################################################################
########################### Script Variables ##########################
############################################################################

#pattern playback rate 240 positions for 360deg
PLAYBACK_LEVEL = 60 # open loop playback gain(?) Hz = 90deg/sec
CL_GAIN_X = -3  # closed loop gain(?). alysha had it set up to -1; Francesca to 3

#Stimulus periods (seconds)
MOTION_DURATION = 0.75*96/(1.0*PLAYBACK_LEVEL)  #3.0
PREMOTION_DURATION = 3.0 # 5.0
POSTMOTION_DURATION = 3.0 # 5.0
FIXATION_DURATION = 5.0 # 5.0

NUM_REPS = 4

# construct the list of motion patterns we will test. Three different
# patterns for each type of motion.
PATTERN_LIST = [['ol_loom_ripple_%s_rep%s'%(d,r) for d in ['front', 'back']]
                        for r in list(range(NUM_REPS))]

PATTERN_LIST = [item for sublist in PATTERN_LIST for item in sublist]
print(PATTERN_LIST)
CONDITION_CLOSED_LOOP = 'cl_stripe2'

# name of unmixer being used (Thad's or Johan's)
UNMIXER_NAME = 'unmixer'  # 'unmixer' or 'live_viewer' 

# pattern position funciton
# x_position_func = 'position_function_5_looming_patterns_0.16.mat' # 'default'
x_position_func = 'position_function_1_looming_patterns_0.01.mat' # 'default'

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
        
        # ----------------------------------------------------------------------
        # get left and right reference frames
        # rospy.wait_for_service('/unmixer_left/RefFrameServer')
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
                      'head_fixed':head_fixed}

        meta_pub.publish(cPickle.dumps(metadata))
         
        # ----------------------------------------------------------------------
        # read out some variables
        conditions = PATTERN_LIST

        ###################################################################################
        # Run experiment
        ###################################################################################

        # get start time
        t0 = time.time()
        
        # stop panels 
        ctrl.stop()

        # loop over conditions
        for condition in np.random.permutation(conditions):
            # print condition
            
            #################################################
            # Closed Loop
            #################################################
            rospy.logwarn('enter closed loop stripe fixation')
            # print 'enter closed loop stripe fixation'

            # publish the type of stimulus to both blk and exp ros channels
            blk_pub.publish('pretrial_stripe_fix')
            exp_pub.publish('closed_loop;gain=%s'%(CL_GAIN_X))
            
#            # run closed loop 
            exc_visual_stim(ctrl, CONDITION_CLOSED_LOOP, FIXATION_DURATION, gain_x=CL_GAIN_X, x_init=0)

            #################################################
            # Open Loop
            #################################################
            rospy.logwarn('enter open loop stimulus presentation: %s'%(condition))
            #print 'enter open loop stimulus presentation: %s'%(condition)
            blk_pub.publish(condition)

            # get portion of condition string WITHOUT rep number -- just corresponds to rotation type (should do this with regex...)
            condition_split = condition.split('_')
            stim_type_str = '_'.join(condition_split[:-1])
            pattern_key = '_'.join(condition_split[1:-1])

            # get initial x value for visual stim 
            x_init = 0 # np.random.randint(0,96)
           

            exc_visual_stim(ctrl, stim_type_str, MOTION_DURATION, gain_x=PLAYBACK_LEVEL, gain_y=0, bias_x=0, bias_y=0, x_init=x_init, pre_motion_duration=PREMOTION_DURATION, post_motion_duration=POSTMOTION_DURATION, x_position_func=x_position_func)
            
            # finally, do post-motion stop period 
            # NB: panels should have stopped moving based on exc_visual_stim above, so just need to pause here
#                ctrl.stop()
#                exp_pub.publish('open_loop;visual;pattern=%s;static'%(condition))
#                time.sleep(POSTMOTION_DURATION)
                
        #################################################
        # Closed Loop (Post trial)
        #################################################
        rospy.logwarn('enter closed loop stripe fixation')
        #print 'enter closed loop stripe fixation'
        # publish the type of stimulus to both blk and exp ros channels
        blk_pub.publish('posttrial_stripe_fix')
        exp_pub.publish('closed_loop;gain=%s'%(CL_GAIN_X))
        
#        # run closed loop 
        exc_visual_stim(ctrl, CONDITION_CLOSED_LOOP, FIXATION_DURATION, gain_x=CL_GAIN_X, x_init=0)
#        

        #################################################
        # Wind down expt
        #################################################
        blk_pub.publish('trials_ended')  

        #publish a refrence frame as a status message to mark the end of the experiment.
        
        print(get_ref_frame_left())
        print(get_ref_frame_right())

        meta_pub.publish(cPickle.dumps(metadata))
        
        # print some stuff at the end to let us know we're done!
        rospy.logwarn('end_of_experiment')
        rospy.logwarn(time.time()-t0)
        # print 'end of experiment'
        # print (time.time()-t0)
        
        turn_off_panels(ctrl)
    except rospy.ROSInterruptException:
        print ('exception')

