import math
import rospy

class CubicSpline:
    def __init__(self, avgspeed=0.2, mintime=0.001):
        # Define all class variables so they are all in one spot.
        self.AVGSPEED = avgspeed
        self.MINTIME = mintime
        self.tmove = mintime
        self.t0 = 0.0  # Start time.
        self.tf = 0.0  # End time
        self.a = 0.0   # Coefficients of the spline.
        self.b = 0.0
        self.c = 0.0
        self.d = 0.0

    def set_avgspeed(self, speed):
        self.AVGSPEED = speed

    def get_tmove(self):
        return self.tmove

    def set_tmove(self, new_tmove):
        self.tmove = new_tmove

    def calc_tmove(self, goalpos, startpos):
        self.tmove = math.fabs(goalpos - startpos) / self.AVGSPEED
        rospy.loginfo("calculated tmove: %f for move from %f to %f" % (self.tmove, goalpos, startpos))

    def update(self, goalpos, startpos, startvel, tstart):
        ''' Compute the new spline parameters. '''

        # Pick a move time: use the time it would take to move the desired
        # distance at 50% of max speed (~1.5 rad/sec).  Also enforce a
        # 1sec min time.  Note this is arbitrary/approximate - we could
        # also compute the fastest possible time or pass as an argument.
        if (self.tmove < self.MINTIME):
            self.tmove = self.MINTIME

        self.t0 = tstart
        self.tf = tstart + self.tmove
        self.a  = startpos
        self.b  = startvel
        self.c  = ( 3.0 * (goalpos - startpos) / self.tmove + 2.0 * startvel) / self.tmove
        self.d  = (-2.0 * (goalpos - startpos) / self.tmove - 3.0 * startvel) / (self.tmove*self.tmove)

        # Report.
        rospy.loginfo("Moving from %6.3f to %6.3f over %5.3fsec" %
                      (startpos, goalpos, self.tmove))

    def get_commands(self, time):
        '''
        Compute the position, velocity, and effort commands to follow the
        current spline. The 'time' parameter is the time since startup, so that
        time - self.t0 is the amount of time into the spline.
        '''

        # Compute the time into the spline to use.
        r = time - self.t0
        r = max(0.0, r)
        r = min(self.tf - self.t0, r)

        # Compute movement commands from the spline.
        cmdpos = self.a + self.b*r + self.c*r*r + self.d*r*r*r
        cmdvel = self.b + 2*self.c*r + 3*self.d*r*r
        cmdtor = 0.0
        done = time >= self.tf

        return [cmdpos, cmdvel, cmdtor, done]
