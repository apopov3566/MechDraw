#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from camera_calibration.calibrator import ChessboardInfo
from camera_calibration.calibrator import Calibrator

class ExtrinsicCalibrator:
    def __init__(self):
        # Initialize the ROS node.
        rospy.init_node('checker_calibrator')

        board = ChessboardInfo()
        board.n_cols = 8
        board.n_rows = 6
        board.dim = 0.0254

        self.calibrator = Calibrator([board])

        # Get the intrinsic calibration info.
        msg = rospy.wait_for_message('/webcam/camera_info', CameraInfo)

        if (msg.K[0] == 0 or msg.K[1] != 0 or msg.K[2] == 0 or
            msg.K[3] != 0 or msg.K[4] == 0 or msg.K[5] == 0 or
            msg.K[6] != 0 or msg.K[7] != 0 or msg.K[8] != 1):
                rospy.logerr('Camera intrinsic parameters formatted wrong')
                rospy.signal_shutdown('Camera incorrectly calibrated')
                return
        self.K = np.float64(msg.K).reshape(3, 3)
        self.D = np.float64(msg.D)

        # Setup the perspective matrix.
        self.perspective_mat = None


    def callback_method(self, image):
        # Test if the checkerboard is in the image.
        gray = self.calibrator.mkgray(image)
        (ok, corners, board) = self.calibrator.get_corners(gray)

        if not ok:
            print('no checkerboard found')
            return
        
        corners = corners.reshape(-1, 2)

        # Set the x, y, z data for each corner, assuming the checkerboard is
        # on the plane z = 0, and centered at x = 0, y = 0.
        xyz = np.zeros((len(corners), 2))

        for r in range(board.n_rows):
            for c in range(board.n_cols):
                i = r * board.n_cols + c
                xyz[i][0] = board.dim * ((board.n_cols - 1) / 2.0 - c)
                xyz[i][1] = board.dim * (r - (board.n_rows - 1) / 2.0)
#                xyz[i][2] = 0.0

        uvlist = corners
        xylist = xyz

#        print('uvlist:', uvlist)
#        print('xylist:', xylist)

        self.update_perspective(uvlist, xylist)
        self.apply_perspective(None)


    def update_perspective(self, uvlist, xylist):
#        self.perspective_mat = cv2.getPerspectiveTransform(uvlist, xylist)
        (self.perspective_mat, _) = cv2.findHomography(uvlist, xylist)
        print('new perspective:', self.perspective_mat)


    def apply_perspective(self, uvlist):
        uvlist = np.float32([320, 240])  # TODO: deleteme

        if self.perspective_mat is None:
            print('perspective mat not setup yet')
            return

        xy = cv2.perspectiveTransform(uvlist.reshape(1, -1, 2),
                                      self.perspective_mat).reshape(2)
        print('got xy:', xy)


    def calibration_loop(self):
        # Subscribe to the camera topic.
        rospy.Subscriber('/webcam/image_raw', Image, callback)

        servo = rospy.Rate(20)

        while not rospy.is_shutdown():
#            print('K:', self.K)
#            print('D:', self.D)
            servo.sleep()


ecal = ExtrinsicCalibrator()

def callback(image):
    ecal.callback_method(image)


if __name__ == '__main__':
    ecal.calibration_loop()
