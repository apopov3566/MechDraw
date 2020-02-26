#!/usr/bin/env python

import numpy as np
import sys
import rospy
import time

from detector import Detector
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from mechdraw.msg import GestureArray, SingleGesture
from multiprocessing import Pool
import cv2

class Detect:
    def __init__(self, nprocs=2):
        self.detectors = [Detector('../cfg/gesture_detection.cfg', '../cfg/gesture_detection.weights')]
        
        self.detect_pub = rospy.Publisher('detections', GestureArray, queue_size=10)
        self.detect_msg = GestureArray()
        self.bridge = CvBridge()
        rospy.init_node('detect')
        self.servo = rospy.Rate(100)

    def inverse_transform(self, detection):
        return detection
        
    def image_callback_method(self, data):

        id = rospy.get_caller_id()
        img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        img = img.astype(np.uint8)
        img = cv2.flip(img, 0)
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), swapRB=True, crop=False)
#        except CvBridgeError as e:
#            print(e)
            
        self.detect_msg.gestures = []
        for detector in self.detectors:
#             prediction = detector.predict(img)
            prediction = detector.predict(blob)
            for detection in prediction:
                detection = self.inverse_transform(detection)
                message = SingleGesture()
                message.xcenter = detection.x
                message.ycenter = detection.y
                message.width = detection.w
                message.height = detection.h
                message.id = detection.det_class
                message.confidence = detection.det_conf
                
                self.detect_msg.gestures.append(message)
            
        #if len(self.detect_msg.gestures) > 0:
        self.detect_pub.publish(self.detect_msg)
            
        
    def start_detection(self):
        rospy.Subscriber('image', Image, image_callback)
        while not rospy.is_shutdown():
            self.servo.sleep()


d = Detect()

def image_callback(data):
    d.image_callback_method(data)

if __name__ == '__main__':
    d.start_detection()
