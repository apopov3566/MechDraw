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
from mechdraw.msg import Goal, Point, Path
from mechdraw.srv import Abort, AbortResponse
from tipmove import TipMove
from jointmove import JointMove
from abortmove import AbortMove
from grabmove import GrabMove

class CmdQueue:
    def __init__(self, cmdpos, cmdvel=[0.0, 0.0, 0.0, 0.0, 0.0]):
        self.q = deque()
        self.cmdpos = cmdpos
        self.cmdvel = cmdvel
        self.setup_front = False  # The move at queue head was already setup.
        self.aborting = False

        assert(len(self.cmdpos) == len(self.cmdvel))


    def enqueue(self, move):
        '''
        Enqueue a movement to perform.

        move (move instance) : The move to perform. This should be an instance
                               of a class that has a get_commands() function
                               which returns positions, velocites, and efforts
                               for the motors.
        '''

        if not self.q:
            # The queue is empty.
            move.setup()

        # Append on the left becuase pop() pops from the right.
        self.q.appendleft(move)


    def abort(self):
        '''
        Abort the current movement. When the abort move is finished, move to
        the last non-aborted position and then continue the interrupted
        movement.
        '''

        # Add the return movement to the front of the queue.
        p1 = self.cmdpos[0]
        p2 = self.cmdpos[1]
        p3 = self.cmdpos[2]
        self.q.append(JointMove(p1, p2, p3))

        # Add the abort movement to the front of the queue.
        self.setup_front = False
        self.q.append(AbortMove())
        

    def get_commands(self, time):
        '''
        Determine the commands for the motors to perform the next movement. If
        there are no more movements to perform, command the robot to hold its
        position.

        time (float) : time in seconds

        '''

        # Check if there is a pending movement.
        if self.q:
            movement = self.d[-1]

            # Make sure this movement has been setup.
            if not self.setup_front:
                movement.setup(self.cmdpos, self.cmdvel, time)
                self.setup_front = True

            motor_cmds = movement.get_commands(time)

            # If the movement has finished and we are not aborting, we should
            # pop this movement, so we can perform the next one.
            if movement.is_done() and not self.aborting:
                self.q.pop()

                # Setup the next movement.
                if self.q:
                    self.q[-1].setup(self.cmdpos, self.cmdvel, time)
                else:
                    # The queue is empty, but when a new movement is added, we
                    # have to set it up.
                    self.setup_front = False
        else:
            motor_cmds = [self.cmdpos, [0.0] * 5, [0.0] * 5]

        # Remember the returned commands for use next iterations.
        if isinstance(movement, GripMove):
            self.cmdpos[4] = motor_cmds[0]
            self.cmdvel[4] = motor_cmds[1]

        else:
            self.cmdpos[0] = motor_cmds[0][0]
            self.cmdpos[1] = motor_cmds[0][1]
            self.cmdpos[2] = motor_cmds[0][2]
            self.cmdpos[3] = motor_cmds[0][2]

            self.cmdvel[0] = motor_cmds[1][0]
            self.cmdvel[1] = motor_cmds[1][1]
            self.cmdvel[2] = motor_cmds[1][2]
            self.cmdvel[3] = motor_cmds[1][2]


        return [self.cmdpos, self.cmdvel, [0.0] * 5]


class Moveto:
    def __init__(self):
        # Initialize the ROS node.
        rospy.init_node('moveto')

        # Create a publisher to send commands to the robot.
        self.pub = rospy.Publisher('/hebiros/robot/command/joint_state',
                                   JointState, queue_size=10)

        self.command_msg = JointState()
        self.command_msg.name = ['Red/3', 'Red/5', 'Red/4', 'Red/7', 'Red/6']

        # Create a servo loop at 100Hz.
        self.servo = rospy.Rate(100)
        self.starttime = rospy.Time.now()

        # Set the initial goals as the current positions.
        msg = rospy.wait_for_message('/hebiros/robot/feedback/joint_state', JointState);
        self.goals = msg.position

        # Keep track of the previous motor commands.
        self.cmdpos = self.goals
        self.cmdvel = [0.0] * 5  # We should be starting with 0 velocity.

        # Keep track of time of last command.
        self.cmd_time = 0.0

        self.cmd_queue = CmdQueue(self.cmdpos)

        rospy.loginfo("Setup servo with dt %f" % self.servo.sleep_dur.to_sec())

    def joint_joint_callback_method(self, data):
        '''
        Update the goal to move the joints to a desired location.
        '''
        id = rospy.get_caller_id()
        rospy.loginfo(id + ' received joint-joint message: ' + str(data))

        c1 = data.position[0]
        c2 = data.position[1]
        c3 = data.position[2]
        c4 = data.position[3]
        c5 = data.position[4]
        self.cmd_queue.enqueue(0, (c1, c2, c3, c4, c5))

    def tip_joint_callback_method(self, data):
        '''
        Update the goal to move the tip through joint space to a goal location.
        '''
        id = rospy.get_caller_id()
        rospy.loginfo(id + ' received tip-joint message: ' + str(data))

        x = data.position[0]
        y = data.position[1]
        z = data.position[2]
        phi = data.position[3]
        grip = data.position[4]
        joints = ikin(np.array([x, y, z, phi]))
        self.cmd_queue.enqueue(1, (joints[0], joints[1], joints[2], joints[3], joints[4]))

    def tip_tip_callback_method(self, data):
        '''
        Update the goal to move the tip through tip space to a goal location.
        '''

        id = rospy.get_caller_id()
        rospy.loginfo(id + ' received tip-tip message: ' + str(data))

        x = data.position[0]
        y = data.position[1]
        z = data.position[2]
        phi = data.position[3]
        grip = data.position[4]

        self.cmd_queue.enqueue(2, (x, y, z, phi, grip))

    def path_callback_method(self, data):
        '''
        Update the goal to move the tip through tip space to a goal location.
        '''

        id = rospy.get_caller_id()
        rospy.loginfo(id + ' received path message: ' + str(data))

        self.cmd_queue.enqueue(3, data.path)

    def abort_callback_method(self, data):
        '''
        Update the goal to move the tip through tip space to a goal location.
        '''

        id = rospy.get_caller_id()
        rospy.loginfo(id + ' received abort message: ' + str(data))

        self.cmd_queue.aborting = data.abort
        self.cmd_queue.abort()

    def send_commands(self):
        # Setup subscribers for each goal message type.
        rospy.Subscriber('goal/joint-joint', Goal, joint_joint_callback)
        rospy.Subscriber('goal/tip-joint', Goal, tip_joint_callback)
        rospy.Subscriber('goal/tip-tip', Goal, tip_tip_callback)
        rospy.Subscriber('goal/path', Path, path_callback)

        rospy.Service('abort', Abort, abort_callback)

        # Update the motors while ROS is still running.
        while not rospy.is_shutdown():
            # Find the current time since starting.
            servotime = rospy.Time.now()
            self.t = (servotime - self.starttime).to_sec()

            # Get the movement commands.
            all_cmds = self.cmd_queue.get_commands(self.t)
            pos_cmds = [p for (p, _, _, _) in all_cmds]
            vel_cmds = [v for (_, v, _, _) in all_cmds]
            tor_cmds = [t for (_, _, t, _) in all_cmds]

            # Build the command message.
            self.command_msg.header.stamp = servotime
            self.command_msg.position = pos_cmds
            self.command_msg.velocity = vel_cmds
            self.command_msg.effort = tor_cmds
            self.command_msg.effort[4] = -3.5 * np.cos(pos_cmds[4])

            # Send the motor commands.
            self.pub.publish(self.command_msg)

#            # Get the movement commands.
#            with self.spline_mutex:
#                cmds = [self.splines[i].get_commands(self.t) for i in range(2)]
#                self.cmdpos = [cmds[0][0], cmds[1][0]]
#                self.cmdvel = [cmds[0][1], cmds[1][1]]
#                cmdtor      = [cmds[0][2], cmds[1][2]]
#
#            rospy.loginfo('pitch: (cmdpos, cmdvel, cmdtor): (%f, %f, %f)',
#                          self.cmdpos[0], self.cmdvel[0], cmdtor[0])
#
#            rospy.loginfo('yaw: (cmdpos, cmdvel, cmdtor): (%f, %f, %f)',
#                          self.cmdpos[1], self.cmdvel[1], cmdtor[1])
#
#            # Command the robot to go to the specified position.
#            self.command_msg.header.stamp = servotime
#            self.command_msg.position = self.cmdpos
#            self.command_msg.velocity = self.cmdvel
#            self.command_msg.effort   = cmdtor
#            self.pub.publish(self.command_msg)

            # Wait a bit
            self.servo.sleep()

# Make a global instance in order to use the callback methods.
m = Moveto()

def joint_joint_callback(data):
    m.joint_joint_callback_method(data)

def tip_joint_callback(data):
    m.tip_joint_callback_method(data)

def tip_tip_callback(data):
    m.tip_tip_callback_method(data)

def path_callback(data):
    m.path_callback_method(data)

def abort_callback(data):
    m.abort_callback_method(data)
    return AbortResponse()

if __name__ == '__main__':
    m.send_commands();
