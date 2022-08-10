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
import readline
import cPickle
import itertools
import yaml
import copy
############################################################################
########################### Metadata Information ###########################
############################################################################

exp_description = \
"""

"""

CL_GAIN_X = 3 #alysha had it set up to -1

fly_dob = 'fly'
genotype_nickname = '22H05-Gal4:UAS-GcAMP7f'
head_fixed = True

#print genotype_nickname


############################################################################
########################### Initialize Experiment ##########################
############################################################################

script_path = os.path.realpath(sys.argv[0])
script_dir = os.path.dirname(script_path)
fly_genotype = ""
#load the script to publish as message
with open(script_path,'rt') as f:
    script_code = f.read() 

#############################################################################
################################ Run experiment #############################
#############################################################################

if __name__ == '__main__':
    try:
        import numpy as np
        #print ('start')
        rospy.init_node('exp_script')
        exp_dir = script_dir
        ctrl = display_ctrl.LedControler()
        ctrl.load_SD_inf(exp_dir + '/firmware/panel_controler/SD.mat')
        
        exp_pub = rospy.Publisher('/exp_scripts/exp_state', 
                                    String,
                                    queue_size = 10)
        meta_pub = rospy.Publisher('/exp_scripts/exp_metadata', 
                                    String,
                                    queue_size = 10)
        blk_pub = rospy.Publisher('/exp_scripts/exp_block',
                                    String,
                                    queue_size = 10)
        ch_pub = rospy.Publisher('/kinefly/flystate2phidgetsanalog/command',
                                    String,
                                    queue_size = 10)

#        rospy.wait_for_service('/unmixer_left/RefFrameServer')
#        get_ref_frame_left = rospy.ServiceProxy('/unmixer_left/RefFrameServer', SrvRefFrame)
#        rospy.wait_for_service('/unmixer_right/RefFrameServer')
#        get_ref_frame_right = rospy.ServiceProxy('/unmixer_right/RefFrameServer', SrvRefFrame)

        # init experiment
        time.sleep(1) # wait for all the publishers to come online


        def exc_cl_yaw(duration, ch=0):

            blk_pub.publish('cl_yaw')
            #print ('cl_yaw')
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            #pattern_name = 'Pattern_rot_axis_5.mat'
            #pattern_name = 'Pattern_bar.mat'
            #ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_pattern_id(4)
            #ctrl.set_pattern_id(1)
            ctrl.set_position(0,0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition= cl_yaw')
            ctrl.start()
            time.sleep(duration)
            ctrl.stop()
            ctrl.send_gain_bias(gain_x = 0, gain_y = 0, bias_x = 0,bias_y = 0)
            ctrl.start()
            time.sleep(0.1)
            ctrl.stop()


        def exc_ol_yaw_right (duration,gain_x,gain_y,bias_x,bias_y,ch=0):

            blk_pub.publish('ol_yaw_right')
            #print ('ol_yaw')
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')

            pattern_name = 'Pattern_rot_axis_5.mat'
            ctrl.set_pattern_by_name(pattern_name)
            #ctrl.set_pattern_id(15)
            #ctrl.set_pattern_id(1)
            ctrl.set_position(0,0)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition= ol_yaw')
            ctrl.start()
            time.sleep(duration)
            ctrl.stop()
            ctrl.send_gain_bias(gain_x = 0, gain_y = 0, bias_x = 0,bias_y = 0)
            ctrl.start()
            time.sleep(0.1)
            ctrl.stop()


        def exc_ol_yaw_left (duration,gain_x,gain_y,bias_x,bias_y,ch=0):

            blk_pub.publish('ol_yaw_left')
            #print ('ol_yaw')
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')

            pattern_name = 'Pattern_rot_axis_4.mat'
            ctrl.set_pattern_by_name(pattern_name)
            #ctrl.set_pattern_id(15)
            #ctrl.set_pattern_id(1)
            ctrl.set_position(0,0)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition= ol_yaw')
            ctrl.start()
            time.sleep(duration)
            ctrl.stop()
            ctrl.send_gain_bias(gain_x = 0, gain_y = 0, bias_x = 0,bias_y = 0)
            ctrl.start()
            time.sleep(0.1)
            ctrl.stop()



        def turn_on_panels (duration):

            blk_pub.publish('all_on')
            ctrl.stop()
            ctrl.all_on()
            time.sleep(duration)
            ctrl.stop()

        def turn_off_panels_d (duration):

            blk_pub.publish('all_on')
            ctrl.stop()
            ctrl.all_off()
            time.sleep(duration)
            ctrl.stop()


        def turn_off_panels ():

            blk_pub.publish('all_off')
            ctrl.stop()
            ctrl.all_off()
            time.sleep(1)
            ctrl.stop()


        def get_exp_param ():
            #param_filepath = '/home/imager/work/muscle_imaging_integration_exp_parameters/MI_062922.yaml'
            param_filepath = '/home/imager/work/muscle_imaging_integration_exp_parameters/MI_062922_2.yaml'

            with open(param_filepath, 'r') as f:
                exp_param = yaml.safe_load(f)
            return exp_param

        def get_trial_params (exp_param, trial_index):
            trial_param = copy.deepcopy(exp_param['trials'][trial_index])
            return trial_param

        ############################################################
        #call get_ref_frame service, this will not only get the current
        #user-defined reference frame but also publish the refrence
        #frame as a message to be logged in rosbag.
        ############################################################

        metadata =   {#'git_SHA':git_SHA,
                      'script_path':script_path,
                      'exp_description':exp_description,
                      'script_code':script_code,
                      'fly_dob':fly_dob,
                      'fly_genotype':fly_genotype,
                      'genotype_nickname':genotype_nickname,
                      'head_fixed':head_fixed}

        meta_pub.publish(cPickle.dumps(metadata))

        ###################################################################################

        #Run experiment

        t0 = time.time()
        ctrl.stop()

        params = get_exp_param ()

        for t in range(params['number_of_trials']):

            rospy.logwarn(t)

            trial_params = get_trial_params (params, t)

            if trial_params['type'] == 'yaw_right_open_loop':

                  duration = trial_params['duration']
                  gain_x = trial_params['gain_x']
                  gain_y = trial_params['gain_y']

                  exc_ol_yaw_right (duration,gain_x,gain_y,0,0,ch=0)

            elif trial_params['type'] == 'yaw_left_open_loop':

                  duration = trial_params['duration']
                  gain_x = trial_params['gain_x']
                  gain_y = trial_params['gain_y']

                  exc_ol_yaw_left (duration,gain_x,gain_y,0,0,ch=0)


            elif trial_params['type'] == 'yaw_closed_loop':

                  duration = trial_params['duration']

                  exc_cl_yaw (duration,ch=0)

            elif trial_params['type'] == 'all_on':

                  duration = trial_params['duration']

                  turn_on_panels (duration)

            elif trial_params['type'] == 'all_off':

                  duration = trial_params['duration']

                  turn_off_panels_d (duration)

            else:

                  turn_off_panels ()
                  rospy.logwarn('end_of_experiment')

        turn_off_panels ()
        rospy.logwarn('end_of_experiment')


        print (time.time()-t0)

    except rospy.ROSInterruptException:
        print ('exception')
