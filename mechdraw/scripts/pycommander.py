#!/usr/bin/env python

import rospy
import numpy as np
from collections import deque
from threading import Lock

from mechdraw.msg import GestureArray, SingleGesture
# from mechdraw.msg import Goal, Point, Path, Grab, Selector
# from mechdraw.srv import Abort, AbortResponse
# from tipmove import TipMove
# from jointmove import JointMove
# from abortmove import AbortMove
# from grabmove import GrabMove
# from markermove import MarkerMove
# from pathmove import PathMove

class Commander:
    def __init__(self):
        # Initialize the ROS node.
        rospy.init_node('commander')

        self.deque = deque(maxlen=10)
        self.deque_mutex = Lock()

    def sort_to_arrays(self, detect, classes):
        detect_array = [[]] * len(classes)
        for detection in detect.gestures:
            x = detection.xcenter
            y = detection.ycenter
            w = detection.width
            h = detection.height
            c = detection.id
    
            detect_array[classes.index(c)].append([x, y, w, h])
    
        return detect_array
    
    def detect_lens(self, detect_arrays, classes):
        d_lens = []
        for detect_array in detect_arrays:
            d_len = []
            for c in detect_array:
                d_len.append(len(c))
            d_lens.append(d_len)
        return np.array(d_lens)
    
    def stable_series(self, series, thresh):
        return np.max(np.std(series, 0)) < thresh
    
    def stable(self, detect_list, thresh = 3, min_len = 5, classes = ['neutral','marker','point','l','o','finger']):
        if len(detect_list) < min_len:
            return False
    
        d_arrays = [self.sort_to_arrays(x, classes) for x in detect_list]
        
        # check same length
        d_lens = self.detect_lens(d_arrays, classes)
        if np.max(np.std(d_lens, 0)) > 0:
            return False
    
        # check stdev
        for c in range(len(classes)):
            for i in range(len(d_arrays[0][c])):
                series = []
                for t in range(len(d_arrays)):
                    series.append(d_arrays[t][c][i])
                if not self.stable_series(np.array(series), thresh):
                    return False
    
        return True 


    def generate_move(self, gestures,
                      ids = ['neutral', 'marker', 'point', 'l', 'o', 'finger']
                     ):
        '''
        Determine the intent of the user given a list of stable gestures. Return
        a tuple consisting of a function and appropriate arguments such that
        calling the function on the given arguments sends commands to the robot
        to perform the user's intention.
    
        gestures (GestureArray) : the array of detected gestures indicating some
                                  intended robot movement
        '''
    
        # Create a dictionary from each gesture id to the detected gestures of
        # that type.
        ordered_gests = {}
        for id in ids:
            ordered_gests[id] = []
    
        for gest in gestures.gestures:
            ordered_gests[gest.id].append(gest)
    
        intents = []
    
        # Check if there are two pointing fingers, signaling a line to be drawn
        # between the points.
        if len(ordered_gests['point']) == 2 and len(ordered_gests['finger']) == 2:
            # Draw a line from the first point to the second.
            points = ordered_gests['point']
            x1 = points[0].xcenter
            y1 = points[0].ycenter
            x2 = points[1].xcenter
            y2 = points[1].ycenter
    
            intents.append(('line', x1, y1, x2, y2))
    
        return intents

    def gesture_callback_method(self, gest_array):
        with self.deque_mutex:
            self.deque.appendleft(gest_array)


    def main_loop(self):
        # Subscribe to the gesture detection topic.
        rospy.Subscriber('detections', GestureArray, gesture_callback)

        # Make a servo to sleep while waiting for gesture callbacks.
        servo  = rospy.Rate(20)

        while not rospy.is_shutdown():
            with self.deque_mutex:
                if self.stable(list(self.deque), thresh=30):
                    print(self.generate_move(self.deque[-1]))
            servo.sleep()


# Make a global class instance to call the callback methods.
c = Commander()

def gesture_callback(data):
    c.gesture_callback_method(data)


if __name__ == '__main__':
    c.main_loop()
