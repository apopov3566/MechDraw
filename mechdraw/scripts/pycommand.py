#!/usr/bin/env python
#
#   pymoveto.py
#
# Command the robot move to a specific location. This can be done with three
# different types of messages sent to this node:
#  1. joint position to joint position
#  2. start tip to end tip in joint space
#  3. start tip to end tip in tip space
#

import numpy as np
import sys
import rospy
import Queue
import time
from collections import namedtuple, deque

from cubicspline import CubicSpline
from kinematics import ikin, fkin
from sensor_msgs.msg import JointState
from mechdraw.msg import Goal, Point, Path, Grab, Selector
from mechdraw.srv import Abort, AbortResponse
from tipmove import TipMove
from jointmove import JointMove
from abortmove import AbortMove
from grabmove import GrabMove
from markermove import MarkerMove
from pathmove import PathMove

marker_x = 0.10
marker_y = -0.84

if __name__ == '__main__':
    rospy.init_node('command')
    goalmsg = Goal()
    grabmsg = Grab()
    time.sleep(1) 
    pub = rospy.Publisher('/goal/tip_tip', Goal, queue_size=10)
    pub2 = rospy.Publisher('/goal/grab', Grab, queue_size=10)
    time.sleep(1)
    #goalmsg.position = [0.08, -0.9, 0.4]


    grabmsg.grab = False
    pub2.publish(grabmsg)

    time.sleep(1)
    
    goalmsg.position = [0.5, -0.5, 0.2]
    pub.publish(goalmsg)

    goalmsg.position = [marker_x, marker_y, 0.4]
    pub.publish(goalmsg)
    
    goalmsg.position = [marker_x, marker_y, 0.21]
    pub.publish(goalmsg)

    time.sleep(1)
    grabmsg.grab = True
    pub2.publish(grabmsg)
   
    time.sleep(1) 
    goalmsg.position = [marker_x, -0.75, 0.4]
    pub.publish(goalmsg)
    
    goalmsg.position = [0.5, -0.5, 0.4]
    pub.publish(goalmsg)
    
    goalmsg.position = [0.5, -0.5, 0.18]
    pub.publish(goalmsg)
    
    goalmsg.position = [0.3, -0.5, 0.18]
    pub.publish(goalmsg)
    
    goalmsg.position = [0.3, -0.3, 0.18]
    pub.publish(goalmsg)
    
    goalmsg.position = [0.5, -0.3, 0.18]
    pub.publish(goalmsg)
    
    goalmsg.position = [0.5, -0.5, 0.18]
    pub.publish(goalmsg)
    
    goalmsg.position = [0.5, -0.5, 0.4]
    pub.publish(goalmsg)
    
    goalmsg.position = [marker_x, marker_y, 0.4]
    pub.publish(goalmsg)
    
    goalmsg.position = [marker_x + 0.005, marker_y, 0.18]
    pub.publish(goalmsg)
    
    goalmsg.position = [marker_x, marker_y, 0.21]
    pub.publish(goalmsg)
    
    time.sleep(1)
    grabmsg.grab = False
    pub2.publish(grabmsg)
   
    time.sleep(1) 
    goalmsg.position = [0.5, -0.5, 0.2]
    pub.publish(goalmsg)
    
