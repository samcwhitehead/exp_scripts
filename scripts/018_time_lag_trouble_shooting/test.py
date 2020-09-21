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
import numpy as np


CHRIMSON_VOLTS = 5
CHRIMSON_CHANNEL = 3


if __name__ == '__main__': 

    print 'start'
    rospy.init_node('exp_script')
    ch_pub = rospy.Publisher('/kinefly/flystate2phidgetsanalog/command',
                                String,
                                queue_size = 10)

    while not rospy.is_shutdown():
        print 'hi'
        ch_pub.publish('set_a30 %s'%(5))
        time.sleep(1)
        print 'lo'
        ch_pub.publish('set_a30 0')
        time.sleep(1)

        
