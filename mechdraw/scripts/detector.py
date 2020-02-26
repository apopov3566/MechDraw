import numpy as np
import sys
import cv2

from pydetectors import DetectionTuple

class Detector:

    def __init__(self, cfgpath, weightpath, classes = ['neutral','marker','point','l','o','finger'], width = 640, height = 480, CONF_THRESH = 0.001):
        self.CONF_THRESH = CONF_THRESH
        self.net = cv2.dnn.readNetFromDarknet(cfgpath, weightpath)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        
        self.classes = classes
        self.width = width
        self.height = height
            
        self.layers = net.getLayerNames()
        self.output_layers = [layers[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        

    def predict(img, blob_size = (416, 416)):
        blob = cv2.dnn.blobFromImage(img, 0.00392, blob_size, swapRB=True, crop=False)
        self.net.setInput(blob)
        layer_outputs = self.net.forward(self.output_layers)
        
        detections = []
        for output in layer_outputs:
        for d in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > CONF_THRESH:
                x, y, w, h = (detection[0:4] * np.array([self.width, self.height, self.width, self.height])).astype('int')
                detections.append(DetectionTuple(det_class=classes[int(class_id)], det_conf=confidence, x=x, y=y, w=w, h=h))
        
        return detections
