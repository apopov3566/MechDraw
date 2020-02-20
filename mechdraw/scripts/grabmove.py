import math
import rospy

class GrabMove:
    angles = {False : 0, True : -1.0}
    def __init__(self, grab_target, speed = 0.5):
        self.grab_target = grab_target
        self.xf = angles[grab_target]
        self.spline = CubicSpline()
        
        
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
