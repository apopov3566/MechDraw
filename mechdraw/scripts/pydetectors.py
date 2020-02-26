import numpy as np
import sys
import rospy
import time

from detector import Detector
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from mechdraw.msg import GestureArray, SingleMessage
from multiprocessing import Pool
import cv2

DetectionTuple = namedtuple('DetectionTuple', ['det_class', 'det_conf', 'x', 'y', 'w', 'h'])

class Detect:
    def __init__(self, nprocs=2):
        self.detectors = [Detector('../cfg/gesture_detection.cfg', '../cfg/gesture_detection.weights')]
        
        self.detect_pub = rospy.Publisher('detections', Detection, queue_size=10)
        self.detect_msg = GestureArray()
        
        self.p = Pool(nprocs)
        
        rospy.init_node('detect')
        
    def inverse_transform(detection):
        return detection
        
    def image_callback_method(self, data):

        id = rospy.get_caller_id()
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        except CvBridgeError as e:
            print(e)
            
        self.detect_msg.gestures = []
        for detector in self.detectors:
            prediction = detector.predict(img)
            for detection in prediction:
                detection = self.inverse_transform(detection)
                message = SingleMessage()
                message.xcenter = detection.x
                message.ycenter = detection.x
                message.width = detection.w
                message.height = detection.h
                message.confidence = detection.det_class
                message.class = detection.det_conf
                
                self.detect_msg.gestures.append(message)
            
        if len(self.detect_msg.gestures) > 0:
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
