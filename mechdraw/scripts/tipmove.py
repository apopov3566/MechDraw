import math
import rospy

from cubicspline import CubicSpline
from kinematics import ikin, fkin

class TipMove:
    def __init__(self, x, y, z, speeds=[0.1,0.1,0.1,0.5]):
        self.xf = [x, y, z]
        self.splines = [CubicSpline(), CubicSpline(), CubicSpline()]
        self.speeds = speeds
        self.done = False
        
        
    def setup(self, x0j, v0j, t0):
        x0 = fkin(x0j)
        v0 = [0.0] * len(x0) # TODO: velocity fkin
        
        for i in range(len(self.splines)):
            self.splines[i].set_avgspeed(self.speeds[i])
            self.splines[i].calc_tmove(x0[i], self.xf[i])
            
        maxtime = max([spline.get_tmove() for spline in self.splines])
        
        for i in range(len(self.splines)):
            self.splines[i].set_tmove(maxtime)
            self.splines[i].update(self.xf[i], x0[i], v0[i], t0)
            
    def get_commands(self, t):
        commands = [spline.get_commands(t) for spline in self.splines]
        
        xc = [0.0] * 3
        vc = [0.0] * 3
        qc = [0.0] * 3
        dones = [False] * 3
        for i in range(len(commands)):
            xc[i], vc[i], qc[i], dones[i] = commands[i]
        
        if not (False in self.dones):
            self.done = True
            
        xcj = ikin(xc)
        vcj = [0.0] * len(xcj) # TODO: velocity ikin
        qcj = [0.0] * len(xcj)
        
        return [xcj, vcj, qcj]
    
    def is_done(self):
        return self.done
        
        
