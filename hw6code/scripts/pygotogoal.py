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
from hw6code.msg import Goal

from threading import Lock

#
#   Global Variables.  We use global variables so the callbacks can
#   see the state and pass information to the main (timer) loop.
#
m_params = []

names = ['Red/3', 'Red/5', 'Red/4', 'Red/7', 'Red/6'] 
AVGSPEEDS = [1.0, 0.2, 0.5 , 0.4, 0.2]

for i in range(5):
    s = {}
    s['t0'] = 0.0		# Cubic spline start time
    s['tf'] = 0.0		# Cubic spline final time
    s['a']  = 0.0		# Cubic spline t^0 parameter
    s['b']  = 0.0		# Cubic spline t^1 parameter
    s['c']  = 0.0		# Cubic spline t^2 parameter
    s['d']  = 0.0		# Cubic spline t^3 parameter

    s['cmdpos'] = 0.0            # Current cmd position (rad)
    s['cmdvel'] = 0.0            # Current cmd velocity (rad/sec)
    s['cmdtor'] = 0.0            # Current cmd torque (Nm)

    m_params.append(s)

t = 0.0

# Use a mutex so that parameters are accessed/read/set atomically.
params_lock = Lock()


#
#   Reset the spline parameters.
#
#   Compute and load the spline parameters
#
def setspline(i, goalpos, startpos, startvel, tstart):

    # Pick a move time: use the time it would take to move the desired
    # distance at 50% of max speed (~1.5 rad/sec).  Also enforce a
    # 1sec min time.  Note this is arbitrary/approximate - we could
    # also compute the fastest possible time or pass as an argument.
    MINTIME  = 1.0
    tmove = math.fabs(goalpos - startpos) / AVGSPEEDS[i]
    if (tmove < MINTIME):
        tmove = MINTIME

    # Set the cubic spline parameters.  Make sure the main code (timer)
    # doesn't access until we're done.
    global m_params
    with params_lock:
        m_params[i]['t0'] = tstart
        m_params[i]['tf'] = tstart + tmove
        m_params[i]['a']  = startpos
        m_params[i]['b']  = startvel
        m_params[i]['c']  = ( 3.0 * (goalpos - startpos) / tmove + 2.0 * startvel) / tmove
        m_params[i]['d']  = (-2.0 * (goalpos - startpos) / tmove - 3.0 * startvel) / (tmove*tmove)

def set_goal(m_msg):
    # Reset the trajectory (cubic spline) parameters to reach the goal
    # starting at the current values.

    print(m_msg)
    global t, m_params
    for i in range(len(names)):
        print(i, m_msg.position[i])
        setspline(i, m_msg.position[i], m_params[i]['cmdpos'],  m_params[i]['cmdvel'], t)


if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pygoto')
    pub = rospy.Publisher('/hebiros/robot/command/joint_state', JointState, queue_size=10)
    rospy.sleep(0.4)

    command_msg = JointState()

    for n in names:
        command_msg.name.append(n)
        command_msg.position.append(0)
        command_msg.velocity.append(0)
        command_msg.effort.append(0)

    cur = rospy.wait_for_message('/hebiros/robot/feedback/joint_state', JointState);
    for i in range(len(names)):
        setspline(i, 0.0, cur.position[i], 0.0, 1.0)

    rospy.Subscriber('/goal', Goal, set_goal)

    servo = rospy.Rate(100)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt %f" % dt)

    starttime = rospy.Time.now()
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Compute the commands.  Make sure the spline parameters are
        # not changed in the middle of the calculations!
        with params_lock:
            # Extend the spline in front of beginning or behind the end.

            for i in range(len(names)):
                if (t <= m_params[i]['t0']):
                    r = 0.0
                elif (t >= m_params[i]['tf']):
                    r = m_params[i]['tf'] - m_params[i]['t0']
                else:
                    r = t  - m_params[i]['t0']
                m_params[i]['cmdpos'] = m_params[i]['a'] + m_params[i]['b']*r + m_params[i]['c']*r*r + m_params[i]['d']*r*r*r
                m_params[i]['cmdvel'] = m_params[i]['b'] + 2.0*m_params[i]['c']*r + 3.0*m_params[i]['d']*r*r
                m_params[i]['cmdtor'] = 0.0
                if names[i] == 'Red/7':
                    m_params[i]['cmdtor'] = -3.0

                command_msg.header.stamp = servotime
                command_msg.position[i]  = m_params[i]['cmdpos']
                command_msg.velocity[i]  = m_params[i]['cmdvel']
                command_msg.effort[i]    = m_params[i]['cmdtor']
        pub.publish(command_msg)

        # Wait for the next turn.
        servo.sleep()
