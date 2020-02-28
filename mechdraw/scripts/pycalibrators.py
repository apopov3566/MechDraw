#!/usr/bin/env python

import numpy as np
import sys
import rospy
import time

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from mechdraw.msg import CalibrationArray, SingleCalibration
import cv2

class Calibrate:
    def __init__(self, calib_int = 1, thresh = 0.4, cfg_dir = '/home/user/hebi_ros_ws/src/mechdraw/cfg'):
        self.calib_int = calib_int
        self.thresh = thresh
        templatelocs = [cfg_dir + '/' + d for d in ['dot1r.png','dot2r.png','dot3r.png','dot4r.png']]
        templates = [cv2.imread(path) for path in templatelocs]
        self.templatec = [cv2.split(t) for t in templates]
        self.sizes = [(t.shape[0], t.shape[1]) for t in templates] 

        self.calib_pub = rospy.Publisher('calibrations', CalibrationArray, queue_size=10)
        self.calib_msg = CalibrationArray()
        
        self.bridge = CvBridge()
        rospy.init_node('calibrate')
        self.servo = rospy.Rate(100)
        self.last_calib = rospy.Time.now()
    
    def image_callback_method(self, data):

        if (rospy.Time.now() - self.last_calib).to_sec() < self.calib_int:
            return

        self.last_calib = rospy.Time.now()

        id = rospy.get_caller_id()
        img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        img = img.astype(np.uint8)
       
        imgc = cv2.split(img)
 
        self.calib_msg.locations = []
        for tc, (w, h) in zip(self.templatec, self.sizes):
            res = np.ones((480 - h + 1,640 - w + 1))
            for i in range(len(imgc)):
                res = np.multiply(res, cv2.matchTemplate(imgc[i],tc[i],cv2.TM_CCOEFF_NORMED))

            loc = np.where(res >= self.thresh)
            pts = np.array(list(zip(*loc[::-1])))

            message = SingleCalibration()
            if pts.shape[0] > 0 and np.max(np.std(pts,0)) < 5:
                pt = np.mean(pts,0).astype(int)
                message.u = pt[0]
                message.v = pt[1] + h
                message.visible = True
            else:
                message.visible = False

            self.calib_msg.locations.append(message)
            
        self.calib_pub.publish(self.calib_msg)
        
    def start_calibration(self):
        rospy.Subscriber('image', Image, image_callback)
        while not rospy.is_shutdown():
            self.servo.sleep()


c = Calibrate()

def image_callback(data):
    c.image_callback_method(data)

if __name__ == '__main__':
    c.start_calibration()
