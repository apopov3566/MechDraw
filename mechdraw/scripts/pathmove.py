import math
import rospy

from kinematics import ikin, fkin
from collections import deque

class PathMove:
    def __init__(self, path, speeds=[0.1,0.1,0.1]):
        self.path = path
        self.speeds = speeds
        self.xq = deque()
        self.vq = deque()

        for i in range(len(path)):
            self.xq.appendleft((path[i].x, path[i].y, path[i].z))
        
        for i in range(len(path) - 1):    
            self.vq.appendleft((path[i].x - path[i+1].x, \
                                path[i].y - path[i+1].y, \
                                path[i].z - path[i+1].z))
        
        self.vq.appendleft((0, 0, 0))
        
        
    def setup(self, x0j, v0j, t0):
        pass
 
    def get_commands(self, t):
        commands = [spline.get_commands(t) for spline in self.splines]
        
        xc = list(self.xq.pop())
        vc = list(self.vq.pop())
        qc = [0.0] * 3
        
        xcj = ikin(xc)
        vcj = [0.0] * len(xcj) # TODO: velocity ikin
        qcj = [0.0] * len(xcj)
        
        return [xcj, vcj, qcj]
    
    def is_done(self):
        return len(self.xq) == 0
        
        
