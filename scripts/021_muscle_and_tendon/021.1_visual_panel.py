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
from scw_panel_lib import turn_off_panels, exc_visual_stim

############################################################################
########################### Metadata Information ###########################
############################################################################

exp_description = \
""" Simultaneous imaging of muscle activity and tendon movement ---

Testing to see if I can image both calcium activity in muscles AND tendon movement

NB: While happending in the same fly, the muscles and tendons are being imaged on separate sides

Legs cut off; head NOT fixed (for now)

"""

fly_dob = '08.08.2022'

fly_genotype = """w[1118]/+[HCS] ; +/(GMR39E01-LexA,GCaMP6f-LexOp) ; +/(sr[md710],UAS-tdTom.S)"""
genotype_nickname = 'C82/HCS'

# fly_genotype = """w[1118]/+[HCS] ; +/(GMR39E01-LexA,GCaMP6f-LexOp) ; +/Mkrs"""
# genotype_nickname = 'C82/HCS-ctrl'
head_fixed = False 
legs_cut = True

print genotype_nickname

############################################################################
########################### Script Variables ##########################
############################################################################

#Stimulus periods
MOTION_DURATION = 3.0
PREMOTION_DURATION = 5.0
POSTMOTION_DURATION = 5.0
FIXATION_DURATION = 5.0

NUM_REPS = 2

#pattern playback rate 240 positions for 360deg
PLAYBACK_LEVEL = 30 # open loop playback gain(?) Hz = 90deg/sec
CL_GAIN_X = -1  # closed loop gain(?). alysha had it set up to -1; Francesca to 3

# construct the list of motion patterns we will test. Three different
# patterns for each type of motion.
PATTERN_LIST = [['ol_pitch_%s_rep%s'%(d,r) for d in ['down','up']]
                        for r in list(range(NUM_REPS))]
PATTERN_LIST.extend([['ol_roll_%s_rep%s'%(d,r) for d in ['left','right']]
                        for r in list(range(NUM_REPS))])
PATTERN_LIST.extend([['ol_yaw_%s_rep%s'%(d,r) for d in ['left','right']]
                        for r in list(range(NUM_REPS))])
PATTERN_LIST = [item for sublist in PATTERN_LIST for item in sublist]

CONDITION_CLOSED_LOOP = 'cl_stripe'
# EPI_LEVEL = 0.125 # Voltage sent to Blue LED for imaging GCaMP

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
        try:
            get_ref_frame_left = rospy.ServiceProxy('/unmixer_left/RefFrameServer', SrvRefFrame)
        except (rospy.ServiceException, rospy.ROSException), e:
            print 'LEFT camera not in use: %s'%(e)
            get_ref_frame_left = lambda *args, **kwargs: None

        try:
            get_ref_frame_right = rospy.ServiceProxy('/unmixer_right/RefFrameServer', SrvRefFrame)
        except (rospy.ServiceException, rospy.ROSException), e:
            print 'RIGHT camera not in use: %s'%(e)
            get_ref_frame_right = lambda *args, **kwargs: None

        time.sleep(1) # wait for all the publishers to come online

        # ----------------------------------------------------------------------
        # save metadata
        print(get_ref_frame_left())
        print(get_ref_frame_right())
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

        # loop over repetitions
        for rep in range(NUM_REPS):
            print rep
            for condition in np.random.permutation(conditions):
                # print condition
                
                #################################################
                # Closed Loop
                #################################################
                print 'enter closed loop stripe fixation'

                # publish the type of stimulus to both blk and exp ros channels
                blk_pub.publish('pretrial_stripe_fix')
                exp_pub.publish('closed_loop;gain=%s'%(CL_GAIN_X))
                
                # run closed loop 
                exc_visual_stim(ctrl, CONDITION_CLOSED_LOOP, FIXATION_DURATION, gain_x=CL_GAIN_X)

                #################################################
                # Open Loop
                #################################################
                print 'enter open loop stimulus presentation: %s'%(condition)
                blk_pub.publish(condition)

                # get portion of condition string WITHOUT rep number -- just corresponds to rotation type (should do this with regex...)
                condition_split = condition.split('_')
                stim_type_str = '_'.join(condition_split[:-1])

                # get initial x value for visual stim 
                x_init = np.random.randint(0,96)
               
                # static pattern to start
                exp_pub.publish('open_loop;visual;pattern=%s;static'%(condition))
                exc_visual_stim(ctrl, stim_type_str, PREMOTION_DURATION, gain_x=0, gain_y=0, bias_x=0, bias_y=0, x_init=x_init)

                # next allow stimulus motion
                exp_pub.publish('open_loop;visual;pattern=%s;motion'%(condition))
                exc_visual_stim(ctrl, stim_type_str, MOTION_DURATION, gain_x=PLAYBACK_LEVEL, gain_y=0, bias_x=0, bias_y=0, x_init=x_init)
                
                # finally, do post-motion stop period 
                # NB: panels should have stopped moving based on exc_visual_stim above, so just need to pause here
                exp_pub.publish('open_loop;visual;pattern=%s;static'%(condition))
                time.sleep(POSTMOTION_DURATION)
                
        #################################################
        # Closed Loop (Post trial)
        #################################################
        print 'enter closed loop stripe fixation'
        # publish the type of stimulus to both blk and exp ros channels
        blk_pub.publish('posttrial_stripe_fix')
        exp_pub.publish('closed_loop;gain=%s'%(CL_GAIN_X))
        
        # run closed loop 
        exc_visual_stim(ctrl, CONDITION_CLOSED_LOOP, FIXATION_DURATION, gain_x=CL_GAIN_X)
        

        #################################################
        # Wind down expt
        #################################################
        blk_pub.publish('trials_ended')  

        #publish a refrence frame as a status message to mark the end of the experiment.
        
        print(get_ref_frame_left())
        print(get_ref_frame_right())

        meta_pub.publish(cPickle.dumps(metadata))
        
        # print some stuff at the end to let us know we're done!
        print 'end of experiment'
        print (time.time()-t0)

    except rospy.ROSInterruptException:
        print ('exception')

