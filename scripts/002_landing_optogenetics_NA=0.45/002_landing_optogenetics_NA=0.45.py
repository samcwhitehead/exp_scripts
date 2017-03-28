#!/usr/bin/env python

import time
import os
import sys

import roslib
roslib.load_manifest('ledpanels')
import rospy

from ledpanels.msg import MsgPanelsCommand
from ledpanels.srv import *
from std_msgs.msg import String
from exp_scripts.msg import MsgExpState
from exp_scripts.msg import MsgExpMetadata

from exp_scripts import git_tools

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
#repo_dirs = [os.path.join(repo_root,d) for d in os.listdir(repo_root)]
#repo_dirs = [d for d in repo_dirs if os.path.isdir(d)]
#repo_dirs.append('/media/imager/FlyDataD/src/muscle_model')
#git_SHA = ''.join([p + ':'+ os.popen('git -C %s rev-parse HEAD'%(p)).read() for p in repo_dirs])
#git_SHA = os.popen('git -C /home/imager/catkin/src/exp_scripts/ rev-parse HEAD').read()
assert git_tools.check_git_status(repo_dirs)
git_SHA = git_tools.get_SHA_keys(repo_dirs)

#####################################################################################
#####################################################################################

class LedControler(object):
    """ convenience class for controlling the led's  lets move this to 
    a module soon"""

    def __init__(self):
        
        self.pub = rospy.Publisher('/ledpanels/command', 
                                    MsgPanelsCommand,
                                    queue_size = 10)
        self.msg = MsgPanelsCommand()

    def clear_args(self):
        self.msg.arg1 = 0;self.msg.arg2 = 0;self.msg.arg3 = 0
        self.msg.arg4 = 0;self.msg.arg5 = 0;self.msg.arg6 = 0

    def set_pattern_id(self,id):
        self.msg.command = 'set_pattern_id'
        self.clear_args()
        self.msg.arg1 = id
        self.pub.publish(self.msg)

    def set_pattern_by_name(self,name):
        idx = self.patstrings.index(name) + 1
        self.set_pattern_id(idx)

    def set_position_function_id(self,channel,id,freq = 50):
        self.msg.command = 'set_posfunc_id'
        self.clear_args()
        self.msg.arg1 = channel+1
        self.msg.arg2 = id
        self.pub.publish(self.msg)
        if channel == 0:
            self.msg.command = 'set_funcx_freq'
            self.clear_args()
            self.msg.arg1 = freq
            self.pub.publish(self.msg)
        else:
            self.msg.command = 'set_funcy_freq'
            self.clear_args()
            self.msg.arg1 = freq
            self.pub.publish(self.msg)

    def set_position(self,index_x, index_y):
        self.msg.command = 'set_position'
        self.clear_args()
        self.msg.arg1 = index_x
        self.msg.arg2 = index_y
        self.pub.publish(self.msg)

    def set_mode(self,mode_x,mode_y):
        mode_x_decode = {'xrate=funcx':0, 
                       'xrate=ch0':1, 
                       'xrate=ch0+idx_funcx':2, 
                       'x=ch2':3, 
                       'x=x0+funcx':4, 
                       'debug':5}
        mode_y_decode = {'yrate=funcy':0, 
                       'yrate=ch0':1, 
                       'yrate=ch0+idx_funcy':2, 
                       'y=ch2':3, 
                       'y=y0+funcy':4, 
                       'debug':5}
        self.msg.command = 'set_mode'
        self.msg.arg1 = mode_x_decode[mode_x]
        self.msg.arg2 = mode_y_decode[mode_y]
        self.pub.publish(self.msg)

    def set_function_by_name(self,channel,name,freq = 50):
        channel = {'X':0,'Y':1}[channel.upper()]
        if name == 'default':
            self.set_position_function_id(channel,0,freq = freq)
            return
        if name.split('_')[0].upper() == 'position'.upper():
            idx = self.funcstrings.index(name)+1
            self.set_position_function_id(channel,idx,freq = freq)

    def load_SD_inf(self,path):
        import scipy.io
        matdata = scipy.io.loadmat(path)
        self.patstrings = [x[0] for x in matdata['SD'][0][0][0][0][0][-1][0]]
        self.funcstrings = [x[0] for x in matdata['SD'][0][0][1][0][0][1][0]]

    def start(self):
        self.msg.command = 'start';self.clear_args();self.pub.publish(self.msg)

    def stop(self):
        self.msg.command = 'stop';self.clear_args();self.pub.publish(self.msg)

    def send_gain_bias(self,gain_x=0, bias_x=0, gain_y=0, bias_y=0):
        self.msg.command = 'send_gain_bias'
        self.msg.arg1 = gain_x
        self.msg.arg2 = bias_x
        self.msg.arg3 = gain_y
        self.msg.arg4 = bias_y
        self.pub.publish(self.msg)

    def all_off(self):
        self.msg.command = 'all_off';self.clear_args();self.pub.publish(self.msg)
    
    def set_ao(self,channel,value = 5000):
        self.msg.command = 'set_ao'
        self.clear_args()
        self.msg.arg1 = channel
        self.msg.arg2 = value
        self.pub.publish(self.msg)

if __name__ == '__main__':
    try:
        import numpy as np
        print 'start'
        rospy.init_node('exp_script')
        exp_dir = script_dir
        ctrl = LedControler()
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

        print get_ref_frame()
        
        # init experiment
        time.sleep(5)
        
        meta_pub.publish(git_SHA = git_SHA,
                         script_path = script_path,
                         exp_description = exp_description,
                         script_code = script_code)
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
    
    	meta_pub.publish(git_SHA = git_SHA,
                         script_path = script_path,
                         exp_description = exp_description,
                         script_code = script_code)

    except rospy.ROSInterruptException:
        print 'exception'
        pass
