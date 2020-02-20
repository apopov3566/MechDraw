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
from pathhandler import PathHandler
from kinematics import ikin, fkin, fkin_twist
from sensor_msgs.msg import JointState
from hw6code.msg import Goal, Point, Path
from hw6code.srv import Abort, AbortResponse

class CmdQueue:
    def __init__(self, cmdpos):
        self.q = deque()
        self.splines = [CubicSpline(), CubicSpline(), CubicSpline(), \
                        CubicSpline(), CubicSpline()]
        self.pathhandler = PathHandler()
        self.cmdvel = [0.0] * 5
        self.cmdpos = cmdpos
        self.setup_new_splines = False

        self.CmdData = namedtuple("CmdData", "mode data")

        self.abort = False
        self.setup_abort_splines = False

        # The desired average speeds of each motor in each message mode. These
        # are used when computing spline speeds. The motor order of each row is
        # consistent with the motor order given in cmd_message.name in the
        # Moveto class.
        self.avgspeeds = [[0.2, 0.4, 0.2, 0.2, 0.2],
                          [0.2, 0.2, 0.2, 0.2, 0.2],
                          [0.1, 0.1, 0.1, 1.0, 1.0]]

        self.safe_position = [0, 0, 0, 0, 0]


        assert(len(self.cmdpos) == len(self.cmdvel))

        # Before commands are enqueued, try to maintain current position.
        print("started at location", cmdpos)
        c1 = cmdpos[0]
        c2 = cmdpos[1]
        c3 = cmdpos[2]
        c4 = cmdpos[3]
        c5 = cmdpos[4]
        self.enqueue(0, (c1, c2, c3, c4, c5))
        print(self.q)

    def enqueue(self, mode, data):
        '''
        Enqueue a goal position to the given coordinates.
        mode (int) : 0 -> move to joint position
                     1 -> move to tip position, through joint space
                     2 -> move to tip position, through tip space
        '''

        self.q.append(self.CmdData(mode, data))

    def get_commands(self, time):
        '''
        Give motor commands based on the front command in the queue. If time
        is past the time of the front command and there are more commands in
        the queue, the front command will be dequeued; otherwise, the front
        command will stay at the front so it can finish its spline movement.

        time (float) : seconds since startup
        '''


        motor_cmds = None
        command = self.q[-1]

        if not self.abort:
            if command.mode == 0 or command.mode == 1:
                # Goal positions are joint goals.
                # Check if we have to setup new splines.
                if not self.setup_new_splines:
                    coords = list(command.data)
                    for i in range(len(self.splines)):
                        self.splines[i].set_avgspeed(self.avgspeeds[command.mode][i])
                        self.splines[i].calc_tmove(coords[i], self.cmdpos[i])
                        self.splines[i].update(coords[i], self.cmdpos[i], \
                                               self.cmdvel[i], time);

                    self.setup_new_splines = True

                # Now just get the commands from the splines.
                motor_cmds = [spline.get_commands(time) for spline in self.splines]
            elif command.mode == 2:
                # Goal positions are tip coords; move through tip space.
                if not self.setup_new_splines:
                    coords = list(command.data)
                    x, y, z, phi, th = fkin_twist([self.cmdpos[3],self.cmdpos[1],self.cmdpos[4],self.cmdpos[0]])
                    currpos = (x, y, z, self.cmdpos[0], self.cmdpos[2])
                    for i in range(len(self.splines)):
                        self.splines[i].set_avgspeed(self.avgspeeds[command.mode][i])
                        self.splines[i].calc_tmove(currpos[i], coords[i])

                    maxtime = max([spline.get_tmove() for spline in self.splines])
                    for i in range(len(self.splines)):
                        self.splines[i].set_tmove(maxtime)
                        self.splines[i].update(coords[i], currpos[i], \
                                               0, time);


                    self.setup_new_splines = True

                # Now just get the commands from the splines.
                position = [spline.get_commands(time) for spline in self.splines]

                pos = [0.0] * 5
                vel = [0.0] * 5
                tor = [0.0] * 5
                pos[3], pos[1], pos[4], pos[0] = ikin(np.array([position[0][0], position[1][0], position[2][0], position[3][0]]))

                motor_cmds = []
                for i in range(5):
                    motor_cmds.append([pos[i],vel[i],tor[i],position[i][3]])

                motor_cmds[0] = position[3]
                motor_cmds[2] = position[4]

            elif command.mode == 3:
                if not self.setup_new_splines:
                    path = list(command.data)
                    self.pathhandler.update(path, time)
                    self.setup_new_splines = True


                target, done = self.pathhandler.get_commands(time)
                pos = list(ikin(np.array(target.x, target.y, target.z, target.phi)))
                vel = [0.0] * 5
                tor = [0.0] * 5
                motor_cmds = []
                for i in range(5):
                    motor_cmds.append([pos[i],vel[i],tor[i],done])

                #motor_cmds[2] = [target.grip, 0, 0, False]

        else:
            if not self.setup_abort_splines:
                self.q.appendleft(self.CmdData(0, self.cmdpos))
                for i in range(len(self.splines)):
                    self.splines[i].set_avgspeed(0.1)
                    self.splines[i].calc_tmove(self.safe_position[i], self.cmdpos[i])
                    self.splines[i].update(self.safe_position[i], self.cmdpos[i], \
                                           self.cmdvel[i], time);

                self.setup_abort_splines = True
            motor_cmds = [spline.get_commands(time) for spline in self.splines]

        # Check if we should dequeue this command.
        if len(self.q) > 1:
            # There is another command waiting to be run.
            done = True
            for (_, _, _, finished) in motor_cmds:
                if not finished:
                    done = False
            if done:
                if not self.setup_abort_splines:
                    # All splines for this command finished.
                    self.q.pop()
                    self.setup_new_splines = False
                elif not self.abort:
                    self.setup_abort_splines = False

        position = []
        velocity = []
        for i in range(len(motor_cmds)):
            position.append(motor_cmds[i][0])
            velocity.append(motor_cmds[i][1])
        self.cmdpos = tuple(position)
        self.cmdvel = tuple(velocity)

        return motor_cmds


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

        self.cmd_queue.abort = data.abort

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
