import math
import rospy

class MarkerMove:
    def __init__(self, marker_target, speed = 0.5):
        self.xf = self.marker_angle(marker_target)
        self.spline = CubicSpline()
        
    def marker_angle(self, marker_target):
        return -(math.pi * 2) * (marker_target - 1) / 8
        
    def setup(self, x0, t0):
        v0 = 0
        
        self.spline.set_avgspeed(speed)
        self.spline.calc_tmove(self.xf, x0)
        self.spline.update(self.xf, x0, v0, t0)
            
    def get_commands(self, t):
        xc, vc, qc, self.done = self.spline.get_commands(t)
        return [xc, vc, qc]
    
    def is_done(self):
        return self.done
