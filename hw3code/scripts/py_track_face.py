#!/usr/bin/env python
#
#   pysendcommands.py
#
#   Continually (at 100Hz!) send commands to the robot
#   (in hebiros_node).

import sys
import rospy
import math
from threading import Lock

from sensor_msgs.msg import JointState
from hw3code.msg import Goal
from opencv_apps.msg import FaceArrayStamped

TRACK_RATE = 0.4
SEEK_RATE = 0.3

MAX = [0.7, math.pi/2]
MIN = [-0.7, -math.pi/2]

goal = [0, 0]
goal_lock = Lock()

m_params = []

for i in range(2):
    s = {}
    s['cmdpos'] = 0.0            # Current cmd position (rad)
    s['cmdvel'] = 0.0            # Current cmd velocity (rad/sec)
    s['cmdtor'] = 0.0            # Current cmd torque (Nm)

    m_params.append(s)

class Face_detector:
    def __init__(self, eye_thresh, missed_thresh):
        self.eye_thresh = eye_thresh
        self.missed_thresh = missed_thresh

        self.eye_misses = eye_thresh
        self.missed_detects = missed_thresh

        self.detected = False
        self.search_start = rospy.Time.now()
        self.cwidth = 640
        self.cheight = 480
        self.facex = 0
        self.facey = 0
        self.params_lock = Lock()

    def update(self,face_msg):
        maxface = None
        for face in face_msg.faces:
            if maxface is None or face.face.height > maxface.face.height:
                maxface = face

        if maxface is not None and maxface.face.height > 50 and (len(maxface.eyes) > 0 or self.eye_misses < self.eye_thresh):
            with self.params_lock:
                if len(maxface.eyes) > 0:
                    self.eye_misses = 0
                else:
                    self.eye_misses += 1

                self.detected = True
                self.missed_detects = 0
                self.facex = maxface.face.x - self.cwidth / 2
                self.facey = maxface.face.y - self.cheight / 2

        else:
            with self.params_lock:
                self.missed_detects += 1

        if self.missed_detects > self.missed_thresh and self.detected:
            with self.params_lock:
                self.eye_misses = self.eye_thresh
                self.detected = False
                self.search_start = rospy.Time.now()

    def get_face(self):
        return (self.facey * 2 / self.cheight, self.facex  * 2 / self.cwidth)

    def get_detected(self):
        with self.params_lock:
            return self.detected

    def get_search_time(self):
        with self.params_lock:
            return (self.search_start - rospy.Time.now()).to_sec()

def set_goal(face_msg, args):
    detector = args
    detector.update(face_msg)
    if detector.get_detected():
        face = detector.get_face()
        with goal_lock:
            goal[0] =  -face[0] * TRACK_RATE
            goal[1] = -face[1] * TRACK_RATE

def seek_goal(i, t):
    if i == 0:
        return 0.4
    else:
        return math.pi / 2 * math.sin(t * SEEK_RATE)


if __name__ == "__main__":
    # Initialize the basic ROS node.
    rospy.init_node('pytrackface')

    # Create a publisher to send commands to the robot.  Add some time
    # for the subscriber to connect so we don't loose initial
    # messages.  Also initialize space for the message data.
    pub = rospy.Publisher('/hebiros/robot/command/joint_state', JointState, queue_size=10)
    rospy.sleep(0.4)

    command_msg = JointState()
    names = ['Red/3', 'Red/2']
    for n in names:
        command_msg.name.append(n)
        command_msg.position.append(0)
        command_msg.velocity.append(0)
        command_msg.effort.append(0)

    # Find the starting position and use as an offset for the sinusoid.
    # This will block, but that's appropriate as we don't want to start
    # until we have this information.  Make sure the joints are in the
    # same order in pydefinerobot as here - else things won't line up!

    # Initialize the parameters and state variables.  Do this before
    # the subscriber is activated (as it may run anytime thereafter).
    goalpos = 0.0

    t = 0.0
    cur = rospy.wait_for_message('/hebiros/robot/feedback/joint_state', JointState);
    for i in range(len(names)):
        m_params[i]['cmdpos'] = cur.position[i]
        m_params[i]['cmdvel'] = 0.0
        m_params[i]['cmdtor'] = 0.0

    # Now that the variables are valid, create/enable the subscriber
    # that (at any time hereafter) may read/update the settings.
    detector = Face_detector(10, 20)
    rospy.Subscriber("/face", FaceArrayStamped, set_goal, (detector))

    # Create and run a servo loop at 100Hz until shutdown.
    servo = rospy.Rate(100)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt %f" % dt)

    starttime = rospy.Time.now()
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Adjust the commands, effectively filtering the goal position
        # into the command position.  Note we only use a single
        # parameter (goalpos) which we read just once, so there is no
        # chance of self-inconsistency.  I.e. we don't need to mutex!
        TIMECONSTANT = 0.3		# Convergence time constant
        LAMBDA       = 1.0/TIMECONSTANT # Convergence rate
        MAXVELOCITY  = 1.5              # Velocity magnitude limit

        for i in range(len(names)):
            goalpos = 0

            cmdpos = m_params[i]['cmdpos']
            cmdvel = m_params[i]['cmdvel']
            cmdtor = m_params[i]['cmdtor']


            if detector.get_detected():
                with goal_lock:
                    goalpos = goal[i] + cmdpos
            else:
                goalpos = seek_goal(i, detector.get_search_time())

            goalpos = min(goalpos, MAX[i])
            goalpos = max(goalpos, MIN[i])


            cmdacc = - 1.4 * LAMBDA * cmdvel - LAMBDA*LAMBDA* (cmdpos - goalpos)
            cmdvel = cmdvel + dt * cmdacc
            if   (cmdvel >  MAXVELOCITY):
                cmdvel =  MAXVELOCITY
            elif (cmdvel < -MAXVELOCITY):
                cmdvel = -MAXVELOCITY
            cmdpos = cmdpos + dt * cmdvel

            cmdtor = 0.0

            m_params[i]['cmdpos'] = cmdpos
            m_params[i]['cmdvel'] = cmdvel
            m_params[i]['cmdtor'] = cmdtor

            # Build and send (publish) the command message.
            command_msg.header.stamp = servotime
            command_msg.position[i]  = cmdpos
            command_msg.velocity[i]  = cmdvel
            command_msg.effort[i]    = cmdtor
        pub.publish(command_msg)

        # Wait for the next turn.
        servo.sleep()
