import math
import rospy

class PathHandler:
    def __init__(self, rate=100):
        self.path = []
        self.t0 = 0
        self.rate = rate

    def update(path, t0):
        self.path = path
        self.t0 = t0

    def get_commands(self, time):
        i = int((time - self.t0) * self.rate)
        if i >= len(self.path):
            return (self.path[-1], True)
        return (self.path[i], False)
