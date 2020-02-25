import math
import rospy

from cubicspline import CubicSpline
angles = {False : 0, True : -1.0}

class GrabMove:
    
    def __init__(self, grab_target, speed=0.5):
        self.grab_target = grab_target
        self.xf = angles[grab_target]
        self.spline = CubicSpline()
        self.speed = speed
        
        
    def setup(self, x0, v0, t0):
        self.spline.set_avgspeed(self.speed)
        self.spline.calc_tmove(self.xf, x0[4])
        self.spline.update(self.xf, x0[4], v0[4], t0)
            
    def get_commands(self, t):
        xc, vc, qc, self.done = self.spline.get_commands(t)
        return [xc, vc, qc]
    
    def is_done(self):
        return self.done
