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
############################################################################
########################### Metadata Information ###########################
############################################################################

exp_description = \
"""
Examine correlations between wing muscles on left-vs-right wing under 
open and closed loop optomotor stablization conditions (left-right bias)
+/- vertical translation.
Practice experiments for Alysha.
"""

NUM_REPS = 3
CL_GAIN_X = -1

CHRIMSON_VOLTS = 5
CHRIMSON_CHANNEL = 3

fly_dob = raw_input('fly DOB:')
genotype_nickname = '22H05-Gal4:UAS-GcAMP6f'
head_fixed = True

print genotype_nickname


############################################################################
########################### Initialize Experiment ##########################
############################################################################

script_path = os.path.realpath(sys.argv[0])
script_dir = os.path.dirname(script_path)
fly_genotype = """w[1118]/+[HCS];P{20XUAS-IVS-GCaMP6f}attP40/+;P{y[+t7.7]w[+mC]=GMR22H05-GAL4}attP2/+"""
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

        rospy.wait_for_service('/unmixer_left/RefFrameServer')
        get_ref_frame_left = rospy.ServiceProxy('/unmixer_left/RefFrameServer', SrvRefFrame)
        rospy.wait_for_service('/unmixer_right/RefFrameServer')
        get_ref_frame_right = rospy.ServiceProxy('/unmixer_right/RefFrameServer', SrvRefFrame)
        # init experiment
        time.sleep(5) # wait for all the publishers to come online
            

        def exc_cl_blocks(block_name,gain_x,gain_y,bias_x,bias_y,ch=0):
            pattern_name = 'Pattern_4x4_blocks_60.mat'
            blk_pub.publish(block_name)
            print block_name

            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)

        def exc_ol_blocks(block_name,gain_x,gain_y,bias_x,bias_y,ch=0):
            pattern_name = 'Pattern_4x4_blocks_60.mat'
            blk_pub.publish(block_name)
            print block_name
            
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_4x4_blocks_60.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)





        def exc_ol_stripe(block_name,gain_x,gain_y,bias_x,bias_y,ch=0):
            pattern_name = 'Pattern_bar.mat'

            blk_pub.publish(block_name)
            print block_name
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_bar.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(18,0)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)

        def exc_ol_expansion_right (block_name,gain_x,gain_y,bias_x,bias_y,ch=0):
            pattern_name = 'Pattern_expansion_1_ADS_mpm.mat'

            blk_pub.publish(block_name)
            print block_name
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_expansion_1_ADS_mpm.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(18,0)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)

            ###60 frames per second -- ~6



        def exc_ol_expansion_left (block_name,gain_x,gain_y,bias_x,bias_y,ch=0):
            pattern_name = 'Pattern_expansion_2_ADS_mpm.mat'

            blk_pub.publish(block_name)
            print block_name
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_expansion_2_ADS_mpm.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(18,0)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)


        def exc_yaw_right (block_name,gain_x,gain_y,bias_x,bias_y,ch=0):
            pattern_name = 'Pattern_yaw_1_ADS_mpm.mat'

            blk_pub.publish(block_name)
            print block_name
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_yaw_1_ADS_mpm.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(18,0)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)

            ###60 frames per second -- ~6



        def exc_yaw_left (block_name,gain_x,gain_y,bias_x,bias_y,ch=0):
            pattern_name = 'Pattern_yaw_2_ADS_mpm.mat'

            blk_pub.publish(block_name)
            print block_name
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_yaw_2_ADS_mpm.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(18,0)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)


        def exc_pitch_up (block_name,gain_x,gain_y,bias_x,bias_y,ch=0):
            pattern_name = 'Pattern_pitch_1_ADS_mpm.mat'

            blk_pub.publish(block_name)
            print block_name
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_pitch_1_ADS_mpm.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            #ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(18,0)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)

            ###60 frames per second -- ~6



        def exc_pitch_down (block_name,gain_x,gain_y,bias_x,bias_y,ch=0):
            pattern_name = 'Pattern_pitch_2_ADS_mpm.mat'

            blk_pub.publish(block_name)
            print block_name
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_pitch_2_ADS_mpm.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            #ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(18,0)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)


        def exc_roll_counterclockwise (block_name,gain_x,gain_y,bias_x,bias_y,ch=0):
            pattern_name = 'Pattern_roll_1_ADS_mpm.mat'

            blk_pub.publish(block_name)
            print block_name
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_roll_1_ADS_mpm.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(18,0)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)


        def exc_roll_clockwise (block_name,gain_x,gain_y,bias_x,bias_y,ch=0):
            pattern_name = 'Pattern_roll_2_ADS_mpm.mat'

            blk_pub.publish(block_name)
            print block_name
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_roll_2_ADS_mpm.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(18,0)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)










        ############################################################
        #call get_ref_frame service, this will not only get the current
        #user-defined reference frame but also publish the refrence
        #frame as a message to be logged in rosbag.
        ############################################################
        #print(get_ref_frame_left())
        #print(get_ref_frame_right())

        metadata =   {'git_SHA':git_SHA,
                      'script_path':script_path,
                      'exp_description':exp_description,
                      'script_code':script_code,
                      'fly_dob':fly_dob,
                      'fly_genotype':fly_genotype,
                      'genotype_nickname':genotype_nickname,
                      'head_fixed':head_fixed}

        meta_pub.publish(cPickle.dumps(metadata))
        ch_pub.publish('set_a30 0')
        ###################################################################################
        ###################################################################################
        #set up experimental conditions
                                                ##gainx       gainy   bias x  biasy  chon
        ctups = [c for c in itertools.product(([CL_GAIN_X]),(0,-4,4),(0,-8,8),([0]),([0,1]))]
        conditions = dict()
        #s='yaw_variations, g_x=%s, g_y=%s, b_x=%s, b_y=%s, ch=%s'

        ### create a dictiionary that is keyed by an int but contains, the function to be used as well as the
        ### parameters to send to the function. 
        ### first item is the function, the second is a descriptive name of the block that will be used for
        ### bookkeeping, the last item is a tuple with the parameters to send to the function.


        ##Commenting these out for now 06.18.19
        #[conditions.update({i:(exc_yaw_left,s%ct,ct)}) for i,ct in enumerate(ctups)]
        last_idx = len(conditions.keys())
        


        conditions[last_idx +0] = (exc_roll_clockwise,'roll_clockwise_actually_pitch, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%( 4, 0), ( 4,  0, 0, 0, 0))  #12
        conditions[last_idx +1] = (exc_roll_counterclockwise,'roll_counterclockwise_actually_pitch, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%(12, 0), (12,  0, 0, 0, 0)) # neg pi coded in pattern .mat

        #conditions[last_idx +2] = (exc_pitch_up,'pitch_up, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%( 0, 4),  ( 0,   4, 0, 0, 0))
        #conditions[last_idx +3] = (exc_pitch_down, 'pitch_down, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%( 0,-4),  ( 0,  -4, 0, 0, 0))


        conditions[last_idx +2] = (exc_pitch_up,'pitch_up_actually_roll, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%( 12, 0),  ( 12,  0, 0, 0, 0))
        conditions[last_idx +3] = (exc_pitch_down, 'pitch_down_actually_roll, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%( 12,0),  ( 12,  0, 0, 0, 0))  #neg pi coded in pattern
        
        #conditions[last_idx +4] = (exc_ol_blocks,'ol_blocks, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=1'%( 0, 0),  ( 0,   0, 0, 0, 1))
        #conditions[last_idx +5] = (exc_ol_stripe,'ol_stripe, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=1'%( 70, 0), ( 70,  0, 0, 0, 1))
        #conditions[last_idx +6] = (exc_ol_stripe,'ol_stripe, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=1'%(-70, 0), (-70,  0, 0, 0, 1))

        #conditions[last_idx +7] = (exc_ol_blocks,'ol_blocks, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%( 12, 0), ( 12,  0, 0, 0, 0))
        #conditions[last_idx +8] = (exc_ol_blocks,'ol_blocks, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%(-12, 0), (-12,  0, 0, 0, 0))
        #conditions[last_idx +9] = (exc_ol_blocks,'ol_blocks, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%( 0, 4),  ( 0,   4, 0, 0, 0))
        #onditions[last_idx +4] = (exc_ol_expansion_left,'expansion_left, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%( 12, 0),  ( 12,  0, 0, 0, 0))
        #conditions[last_idx +5] = (exc_ol_expansion_right,'expansion_right, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%( -12, 0),  ( -12,   0, 0, 0, 0))


        conditions[last_idx +6] = (exc_ol_stripe,'ol_stripe, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%( 70, 0), ( 70,  0, 0, 0, 0))
        conditions[last_idx +7] = (exc_ol_stripe,'ol_stripe, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%(-70, 0), (-70,  0, 0, 0, 0))

        conditions[last_idx +8] = (exc_yaw_left,'yaw_left, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%( 4, 0), ( 4,  0, 0, 0, 0))
        conditions[last_idx +9] = (exc_yaw_right,'yaw_right, g_x=%s, g_y=%s, b_x=0, b_y=0, ch=0'%(4, 0), (4,  0, 0, 0, 0))

        ###################################################################################
        ###################################################################################

        #Run experiment
        t0 = time.time()
        print 'enter closed loop stripe fixation'
        gain_x = -1  #-1 #70 #12
        ctrl.stop()
        ctrl.set_pattern_by_name('Pattern_bar.mat')
        ctrl.set_position(np.random.randint(0,96),0)
        ctrl.send_gain_bias(gain_x = gain_x,bias_x = 0.0,gain_y = 0,)  #
        ctrl.set_mode('xrate=ch0','yrate=funcy')
        blk_pub.publish('pretrial_stripe_fix')
        ctrl.start()
        ### publish the state
        exp_pub.publish('condition=pretrial_closed_loop;gain=%s'%(gain_x))
        time.sleep(10)

        for rep in range(NUM_REPS):
            print rep
            for key in np.random.permutation(conditions.keys()):
                condition = conditions[key]
                condition[0](condition[1],*condition[2])

        ctrl.stop()
        ctrl.set_pattern_by_name('Pattern_bar.mat')
        ctrl.set_position(np.random.randint(0,96),0)
        ctrl.send_gain_bias(gain_x = gain_x,bias_x = 0.0,gain_y = 0,)
        ctrl.set_mode('xrate=ch0','yrate=funcy')
        blk_pub.publish('pretrial_stripe_fix')
        ctrl.start()
        ### publish the state


        exp_pub.publish('condition=posttrial_closed_loop;gain=%s'%(gain_x))
        time.sleep(10)

        blk_pub.publish('trials_ended')  
        #publish a refrence frame as a status message to mark the end of the experiment.
        print(get_ref_frame_left())
        print(get_ref_frame_right())

        meta_pub.publish(cPickle.dumps(metadata))

        print time.time()-t0
    except rospy.ROSInterruptException:
        print 'exception'


print 'experiment done'

      #
      #  exp_pub.publish('condition=posttrial_closed_loop;gain=%s'%(gain_x))
      #  time.sleep(10)

      #  blk_pub.publish('trials_ended')  
        #publish a refrence frame as a status message to mark the end of the experiment.
      #  print(get_ref_frame_left())
      #  print(get_ref_frame_right())

      #  meta_pub.publish(cPickle.dumps(metadata))

      #  print time.time()-t0
    #except rospy.ROSInterruptException:
      #  print 'exception'
      