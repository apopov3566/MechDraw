from cubicspline import CubicSpline

class JointMove:
    def __init__(self, p1, p2, p3, speeds=[0.2, 0.2, 0.2]):
        self.goalpos = (p1, p2, p3)
        self.speeds = speeds
        self.splines = [CubicSpline(), CubicSpline(), CubicSpline()]
        self.done = False  # This move has not finished yet.


    def setup(self, x0, v0, t0):
        '''
        Setup the splines to go to the position given on init.

        x0 (float list) : the current motor positions
        v0 (float list) : the current motor velocities
        t0 (float) : the current time in seconds
        '''

        for i in range(len(self.splines)):
            self.splines[i].set_avgspeed(self.speeds[i])
            self.splines[i].calc_tmove(self.goalpos[i], x0[i])
            self.splines[i].update(self.goalpos[i], x0[i], v0[i], t0)


    def get_commands(self, t):
        '''
        Get commands for the motors.

        time (float) : the current time in seconds
        '''

        # Get the commands from the 
        raw_cmds = [spline.get_commands(t) for spline in self.splines]

        # Check if all splines have finished.
        self.done = all(done for (_, _, _, done) in raw_cmds)

        # Rearange the commands.
        cmds = [p for (p, _, _, _) in raw_cmds,
                v for (_, v, _, _) in raw_cmds,
                e for (_, _, e, _) in raw_cmds]

        return cmds


    def is_done(self):
        return self.done