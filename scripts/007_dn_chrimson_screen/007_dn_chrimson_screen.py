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
import split_lines as sl
############################################################################
########################### Metadata Information ###########################
############################################################################

exp_description = \
"""
CsCrimson screen of the collection of DN split lines from Gwenith and Wyatt.
"""
fly_dob = raw_input('fly DOB:')
s_number = raw_input('S-number:')
genotype_nickname = s_number + ' X C-85'
print genotype_nickname
fly_genotype = sl.expand_snum(s_number)
print fly_genotype
pulse_voltages = [ 0.1, 0.5, 1.0, 2.0, 3.0, 5.0]

ol_static_pre_s = 3.0
ol_static_post_s = 3.0
ol_motion_s = 1.0

cs_stim_pre_s = 3.0
cs_stim_pulse = 0.1
cs_stim_post_s = 3.9

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
        ctrl.load_SD_inf(exp_dir + '/firmware/panel_controler/SD.mat')
        
        exp_pub = rospy.Publisher('/exp_scripts/exp_state', 
                                    MsgExpState,
                                    queue_size = 10)
        exp_msg = MsgExpState()
        meta_pub = rospy.Publisher('/exp_scripts/exp_metadata', 
                                    MsgExpMetadata,
                                    queue_size = 10)

        blk_pub = rospy.Publisher('/exp_scripts/exp_block',
                                    String,
                                    queue_size = 10)

        rospy.wait_for_service('/unmixer_left/RefFrameServer')
        get_ref_frame_left = rospy.ServiceProxy('/unmixer_left/RefFrameServer', SrvRefFrame)
        rospy.wait_for_service('/unmixer_right/RefFrameServer')
        get_ref_frame_right = rospy.ServiceProxy('/unmixer_right/RefFrameServer', SrvRefFrame)
        # init experiment
        time.sleep(5) # wait for all the publishers to come online

        def exc_cl(pattern_name,pattern_index,block_name):
            import random
            pattern_name = pattern_name%(random.choice(pattern_index))
            gain_x = -90
            bias_y = 2*48/5.0 #180 deg / sec
            chrimson_level = 3.0
            blk_pub.publish(block_name)
            print block_name
            ctrl.stop()
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_function_by_name('Y','default',freq=50)
            ctrl.send_gain_bias(gain_x = gain_x,bias_x = 0.0,gain_y = 0,bias_y = bias_y)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.start()
            #############
            ctrl.set_ao(3,0)
            ### publish the state
            exp_pub.publish(state = 'condition=cl_stripe;loop=closed;gain_x=%s;bias_y=%s;ao_level=%s;pattern=%s'%(gain_x,bias_y,0,pattern_name))
            time.sleep(3)
            #############
            ctrl.set_ao(3,chrimson_level)
            ### publish the state
            exp_pub.publish(state = 'condition=cl_stripe;loop=closed;gain_x=%s;bias_y=%s;ao_level=%s;pattern=%s'%(gain_x,bias_y,chrimson_level,pattern_name))
            time.sleep(3)
            #############
            ctrl.set_ao(3,0)
            ### publish the state
            exp_pub.publish(state = 'condition=cl_stripe;loop=closed;gain_x=%s;bias_y=%s;ao_level=%s;pattern=%s'%(gain_x,bias_y,0,pattern_name))
            time.sleep(3)
            print 'enter closed loop stripe fixation'
            ctrl.stop()
            ctrl.set_pattern_by_name('Pattern_bar.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_function_by_name('Y','default',freq=50)
            ctrl.send_gain_bias(gain_x = gain_x,bias_x = 0.0,gain_y = 0,)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.start()
            ### publish the state
            exp_pub.publish(state = 'condition=intertrial_closed_loop;gain=%s;ao_level=%s'%(gain_x,0))
            time.sleep(5)

        def exc_ol_expand(pattern_name,x_pos,block_name):
            import random
            #pattern_name = pattern_name%(random.choice(pattern_index))
            gain_x = -90 #azimuth
            bias_y = 2*24/5.0 #set's the expansion rate (25fps) *7.5 deg per frame = 180deg/sec
            chrimson_level = 3.0
            blk_pub.publish(block_name)
            ############################
            print block_name
            ctrl.stop()
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_function_by_name('X','default',freq=50)
            ctrl.set_function_by_name('Y','default',freq=50)
            ctrl.set_position(x_pos,0)
            exp_pub.publish(state = 'condition=ol_expand;loop=open;pattern=%s,bias_y=%s'%(pattern_name,0))
            time.sleep(4.0)
            ctrl.send_gain_bias(gain_x = 0,bias_x=0,gain_y = 0,bias_y = bias_y)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            exp_pub.publish(state = 'condition=ol_expand;loop=open;pattern=%s,bias_y=%s'%(pattern_name,bias_y))
            ctrl.start()
            time.sleep(1.0)
            ctrl.stop()
            exp_pub.publish(state = 'condition=ol_expand;loop=open;pattern=%s,bias_y=%s'%(pattern_name,0))
            time.sleep(4.0)
            ############################
            print 'enter closed loop stripe fixation'
            ctrl.stop()
            ctrl.set_pattern_by_name('Pattern_bar.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_function_by_name('Y','default',freq=50)
            ctrl.send_gain_bias(gain_x = gain_x,bias_x = 0.0,gain_y = 0,bias_y = 0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.start()
            ### publish the state
            exp_pub.publish(state = 'condition=intertrial_closed_loop;gain=%s;ao_level=%s'%(gain_x,0))
            time.sleep(5)

        def exc_ol_stripe(pattern_name,direction,x_pos,block_name):
            import random
            #pattern_name = pattern_name%(random.choice(pattern_index))
            gain_x = -90
            blk_pub.publish(block_name)
            ############################
            print block_name
            ctrl.stop()
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_function_by_name('X','default',freq=50)
            ctrl.set_function_by_name('Y','default',freq=50)
            ctrl.set_position(x_pos,0)
            exp_pub.publish(state = 'condition=ol_stripe;loop=open;pattern=%s'%(pattern_name))
            time.sleep(ol_static_pre_s)
            ctrl.send_gain_bias(gain_x = 0,gain_y = 0,bias_x = 2*49/5.0*direction)
            ctrl.set_mode('xrate=funcx','y=y0+funcy')
            ctrl.start()
            time.sleep(ol_motion_s)
            ctrl.stop()
            time.sleep(ol_static_pre_s)
            ############################
            print 'enter closed loop stripe fixation'
            ctrl.stop()
            ctrl.set_pattern_by_name('Pattern_bar.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_function_by_name('Y','default',freq=50)
            ctrl.send_gain_bias(gain_x = gain_x,bias_x = 0.0,gain_y = 0,bias_y=0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.start()
            ### publish the state
            exp_pub.publish(state = 'condition=intertrial_closed_loop;gain=%s;ao_level=%s'%(gain_x,0))
            time.sleep(5)

        def exc_ol_full(pattern_name,direction,x_pos,block_name):
            import random
            pattern_index = (1,2,3)
            pattern_name = pattern_name%(random.choice(pattern_index))
            gain_x = -90
            blk_pub.publish(block_name)
            ############################ 
            print block_name
            ctrl.stop()
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_function_by_name('X','default',freq=50)
            ctrl.set_function_by_name('Y','default',freq=50)
            ctrl.set_position(x_pos,0)
            exp_pub.publish(state = 'condition=ol_full;loop=open;pattern=%s'%(pattern_name))
            time.sleep(ol_static_pre_s)
            ctrl.send_gain_bias(gain_x = 0,gain_y = 0,bias_x = 2*49/5.0*direction)
            ctrl.set_mode('xrate=funcx','y=y0+funcy')
            ctrl.start()
            time.sleep(ol_motion_s)
            ctrl.stop()
            time.sleep(ol_static_pre_s)
            ############################ 
            print 'enter closed loop stripe fixation'
            ctrl.stop()
            ctrl.set_pattern_by_name('Pattern_bar.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_function_by_name('Y','default',freq=50)
            ctrl.send_gain_bias(gain_x = gain_x,bias_x = 0.0,gain_y = 0,bias_y=0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.start()
            ### publish the state
            exp_pub.publish(state = 'condition=intertrial_closed_loop;gain=%s;ao_level=%s'%(gain_x,0))
            time.sleep(5)

        def exc_cs_stim(power_level,block_name):
            blk_pub.publish(block_name)
            print block_name
            gain_x = -90
            gain_y = 30
            print 'all panels on'
            ctrl.stop()
            ctrl.all_on()
            exp_pub.publish(state = 'open_loop;opto;ao_level=%s;pulse_off'%(0))
            time.sleep(cs_stim_pre_s)
            exp_pub.publish(state = 'open_loop;opto;ao_level=%s;pulse_on'%(power_level))
            ctrl.set_ao(3,float(power_level))
            time.sleep(cs_stim_pulse)
            exp_pub.publish(state = 'open_loop;opto;ao_level=%s;pulse_off'%(0))
            ctrl.set_ao(3,0)
            time.sleep(cs_stim_post_s)
            print 'enter closed loop stripe fixation'
            ctrl.set_pattern_by_name('Pattern_bar.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_function_by_name('Y','default',freq=50)
            ctrl.send_gain_bias(gain_x = gain_x,bias_x = 0.0,gain_y = 0,bias_y=0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.start()
            ### publish the state
            exp_pub.publish(state = 'condition=intertrial_closed_loop;gain=%s;ao_level=%s'%(gain_x,0))
            time.sleep(30)

        ############################################################
        #call get_ref_frame service, this will not only get the current
        #user-defined reference frame but also publish the refrence
        #frame as a message to be logged in rosbag.
        ############################################################
        print(get_ref_frame_left())
        print(get_ref_frame_right())
        meta_pub.publish(git_SHA = git_SHA,
                         script_path = script_path,
                         exp_description = exp_description,
                         script_code = script_code,
                         fly_dob = fly_dob,
                         fly_genotype = fly_genotype,
                         genotype_nickname = genotype_nickname)
        
        # Opted against these stimuli - in the intrest of time - Employ these
        # in a different experiment. Instead include a full field yaw control.
        #'cl_stripe':       (exc_cl,('Pattern_bar%s.mat',('',),'cl_stripe')),
        #'cl_optomotor':    (exc_cl,('Pattern_full_%s.mat',(1,2,3),'cl_optomotor')),
        #'cl_translation':  (exc_cl,('Pattern_contract_%s.mat',(1,2,3),'cl_translation')),
        #'cl_figure':       (exc_cl,('Pattern_fourier_bar_%s.mat',(1,2,3),'cl_figure')),

        conditions = {'ol_expand_center':(exc_ol_expand,('Pattern_expand.mat',21,'ol_expand_center')),
                      'ol_expand_left':  (exc_ol_expand,('Pattern_expand.mat',93,'ol_expand_left')),
                      'ol_expand_right': (exc_ol_expand,('Pattern_expand.mat',45,'ol_expand_right')),

                      'ol_stripe_left':  (exc_ol_stripe,('Pattern_bar.mat',-1,90,'ol_stripe_left')),
                      'ol_stripe_right': (exc_ol_stripe,('Pattern_bar.mat',1,42,'ol_stripe_right')),

                      'ol_full_left':    (exc_ol_full,('Pattern_full_%s.mat',-1,90,'ol_stripe_left')),
                      'ol_full_right':   (exc_ol_full,('Pattern_full_%s.mat',1,42,'ol_stripe_right')),

                      'cs_stim_power_0': (exc_cs_stim,(pulse_voltages[0],'cs_stim_power_1')),
                      'cs_stim_power_1': (exc_cs_stim,(pulse_voltages[1],'cs_stim_power_2')),
                      'cs_stim_power_3': (exc_cs_stim,(pulse_voltages[2],'cs_stim_power_3')),
                      'cs_stim_power_4': (exc_cs_stim,(pulse_voltages[3],'cs_stim_power_4')),
                      'cs_stim_power_5': (exc_cs_stim,(pulse_voltages[4],'cs_stim_power_5')),
                      'cs_stim_power_6': (exc_cs_stim,(pulse_voltages[5],'cs_stim_power_6'))}

        #Run experiment
        t0 = time.time()
        for rep in range(5):
            print rep
            for key in np.random.permutation(conditions.keys()):
                condition = conditions[key]
                condition[0](*condition[1])    
        #publish a refrence frame as a status message to mark the end of the experiment.
        print(get_ref_frame_left())
        print(get_ref_frame_right())
        meta_pub.publish(git_SHA = git_SHA,
                         script_path = script_path,
                         exp_description = exp_description,
                         script_code = script_code)
        print time.time()-t0
    except rospy.ROSInterruptException:
        print 'exception'