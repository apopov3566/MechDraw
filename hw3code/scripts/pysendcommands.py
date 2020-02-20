#!/usr/bin/env python
#
#   pysendcommands.py
#
#   Continually (at 100Hz!) send commands to the robot
#   (in hebiros_node).

import sys
import rospy
import math

from sensor_msgs.msg import JointState

returned = False
rate = 0.02
# Create a publisher to send commands to the robot.  Also
# initialize space for the message data.
pub = rospy.Publisher('/hebiros/robot/command/joint_state', JointState, queue_size=100)


command_msg = JointState()
command_msg.name.append('Red/3')
command_msg.position.append(0)
command_msg.velocity.append(0)
command_msg.effort.append(0)
command_msg.name.append('Red/2')
command_msg.position.append(0)
command_msg.velocity.append(0)
command_msg.effort.append(0)


def return_to_0(feedback_msg, args):
    global returned, pub
    target = args[0]
    rate = args[1]
    pos = feedback_msg.position

    #print(pos)
    returned = True
    for i in range(len(target)):
        command_msg.position[i] = pos[i]
        command_msg.velocity[i] = 0
        if abs(pos[i] - (target[i])) > 0.01:
            returned = False
            if pos[i] > target[i]:
                command_msg.position[i] = (pos[i] - rate)
            else:
        	    command_msg.position[i] = (pos[i] + rate)
            command_msg.effort[i] = 0
    	pub.publish(command_msg)

    if returned:
        sub.unregister()


if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pysendcommands')


    # Create a servo loop at 100Hz.
    servo = rospy.Rate(100)
    dt    = servo.sleep_dur.to_sec()

    # Run the servo loop until shutdown.
    rospy.loginfo("Running the servo loop with dt %f" % dt)

    global sub
    sub = rospy.Subscriber('/hebiros/robot/feedback/joint_state', JointState, return_to_0, ([0.3, math.pi / 2], 0.05))

    while not returned and not rospy.is_shutdown():
        servo.sleep()

    starttime = rospy.Time.now()
    while not rospy.is_shutdown():
        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Compute the commands.
        cmdtor = 0.0

        # Build and send (publish) the command message.
        command_msg.header.stamp = servotime
        command_msg.position[1]  = math.pi / 2 * math.cos(2*math.pi*t * rate)
        command_msg.velocity[1]  = -math.pi / 2 * math.sin(2*math.pi*t * rate)
        command_msg.effort[1]    = 0
        command_msg.position[0]  = 0.4 * math.sin(2*math.pi*t * 5 * rate) + 0.3
        command_msg.velocity[0]  = 0.4 * math.cos(2*math.pi*t * 5 *rate)
        command_msg.effort[0]    = 0
        pub.publish(command_msg)

        # Wait for the next turn.
        servo.sleep()
