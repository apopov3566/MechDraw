#
#   gravity.py
#
#   Gravity Torque Computation
#
import math


#
#   Gravity Torque
#
def gravity(pos):
    # Set the torque offsets - these should really be cleared by
    # reseting the actuator force sensors...
    off = [0.0] * 5

    # Compute from the tip inward
    grav = [0.0] * 5
    grav[4] = -3.7 * math.cos(pos[4])

    # Return the gravity vector
    return [g + o for (g, o) in zip(grav, off)]
