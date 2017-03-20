#!/usr/bin/env python

#!/usr/bin/env python
# license removed for brevity
import roslib
roslib.load_manifest('ledpanels')
import rospy
import time
from ledpanels.msg import MsgPanelsCommand
from ledpanels.srv import *
from std_msgs.msg import String
from exp_scripts.msg import MsgExpState

class LedControler(object):
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

if __name__ == '__main__':
    try:
        import numpy as np
        print 'start'
        rospy.init_node('exp_script')
        exp_dir = '/media/imager/FlyDataD/Projects/001_landing/'
        ctrl = LedControler()
        ctrl.load_SD_inf(exp_dir + 'SD.mat')
        
        exp_pub = rospy.Publisher('/exp_scripts/exp_state', 
                                    MsgExpState,
                                    queue_size = 10)
        exp_msg = MsgExpState()
        
        # init experiment
        time.sleep(5)
        for rep in range(10):
            for fid in np.random.permutation(len(ctrl.funcstrings)):
                print 'enter closed loop'
                ctrl.stop()
                ctrl.set_pattern_by_name('Pattern_fixation_4_wide_4X12_Pan.mat')
                ctrl.set_position(0,0)
                #ctrl.set_function_by_name('X','default',freq=50)
                ctrl.set_function_by_name('Y','default',freq=50)
                ctrl.send_gain_bias(gain_x = -90,bias_x = 0.0)
                ctrl.set_mode('xrate=ch0','yrate=funcy')
                ctrl.start()
                ### publish the state
                fstring = ctrl.funcstrings[fid]
                exp_msg.state = 'closed_loop, gain = -5'
                exp_pub.publish(exp_msg)
                time.sleep(5)

                print 'enter open loop'                
                ctrl.stop()
                ctrl.set_pattern_by_name('Pattern_linear_expansion_48_Pan.mat')
                ctrl.set_position(0,20)
                ctrl.set_function_by_name('X',fstring,freq=200)
                ctrl.set_function_by_name('Y','default',freq=200)
                ctrl.send_gain_bias(gain_x = 1,gain_y = 0)
                ctrl.set_mode('x=x0+funcx','yrate=funcy')
                ctrl.start()
                ### publish the state
                fstring = ctrl.funcstrings[fid]
                exp_msg.state = 'open_loop, %s'%str(fstring)
                exp_pub.publish(exp_msg)
                time.sleep(4.5)
                print rep

        ctrl.set_position(0,20)
        ctrl.stop()
    except rospy.ROSInterruptException:
        print 'exception'
        pass            
