import math
import rospy

from cubicspline import CubicSpline
from kinematics import ikin, fkin

class AbortMove:
    def __init__(self, speeds=[0.1,0.1,0.1], safepos=[0.5, 0, 0.4]):
        self.xf = safepos # [x, y, z]
        self.vsplines = [CubicSpline(), CubicSpline(), CubicSpline()]
        self.hsplines = [CubicSpline(), CubicSpline(), CubicSpline()]
        self.speeds = speeds
        self.done = False
        
        
    def setup(self, x0j, v0j, t0):
        x0 = list(fkin(x0j))
        v0 = [0.0] * len(x0) # TODO: velocity fkin
        
        xi = x0
        xi[2] = 0.4
        vi = [0.0] * len(xi)
        
        for i in range(len(self.vsplines)):
            self.vsplines[i].set_avgspeed(self.speeds[i])
            self.vsplines[i].calc_tmove(x0[i], xi[i])
        maxtimev = max([spline.get_tmove() for spline in self.vsplines])
        for i in range(len(self.vsplines)):
            self.vsplines[i].set_tmove(maxtimev)
            self.vsplines[i].update(xi[i], x0[i], v0[i], t0)
            
        for i in range(len(self.hsplines)):
            self.hsplines[i].set_avgspeed(self.speeds[i])
            self.hsplines[i].calc_tmove(xi[i], self.xf[i])
        maxtimeh = max([spline.get_tmove() for spline in self.hsplines])
        for i in range(len(self.hsplines)):
            self.hsplines[i].set_tmove(maxtimeh)
            self.hsplines[i].update(self.xf[i], xi[i], vi[i], t0 + maxtimev)
            
    def get_commands(self, t):
        vcommands = [spline.get_commands(t) for spline in self.vsplines]
        hcommands = [spline.get_commands(t) for spline in self.hsplines]
        
        xc = [0.0] * 3
        vc = [0.0] * 3
        qc = [0.0] * 3
        dones = [False] * 3
        
        if (False in [c[3] for c in vcommands]): #doing vmove
            for i in range(len(vcommands)):
                xc[i], vc[i], qc[i], dones[i] = vcommands[i]
            
        else: # doing hmove
            for i in range(len(hcommands)):
                xc[i], vc[i], qc[i], dones[i] = hcommands[i]
                if not (False in dones):
                    self.done = True
            
        xcj = ikin(xc)
        vcj = [0.0] * len(xcj) # TODO: velocity ikin
        qcj = [0.0] * len(xcj)
        
        return [xcj, vcj, qcj]
    
    def is_done(self):
        return self.done
        
        
