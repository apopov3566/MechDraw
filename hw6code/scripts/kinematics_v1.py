import numpy as np

# Define all link lengths 
l2x = 0.49848
l2y = -0.01447
l2z = 0.0748
l3x = 0.48835
l3y = 0.02363
l3z = 0.15973
l4x = 0.98683
l4y = 0.00916
l4z = 0.19078
ltx = 0.98683
lty = 0.00916     # Need to Measure
ltz = 0.19078
'''
l2x = 0.49848
l2y = 0.0
l3x = 0.48835
l3y = 0.0
l4x = 0.98683
l4y = 0.0
l2z = 0.0748
l3z = 0.15973
l4z = 0.19078
'''

def ikin(tip):
    x, y, z, phi = tip[0], tip[1], tip[2], tip[3]
    # x, y, z, phi = tip[0], tip[1], tip[2], tip[3]
    # Compute theta3 to get desired z
    theta3 = np.arcsin(-(z - l4z) / (l4x - l3x) )
    theta4 = -phi + theta3
    
    # Compute important (x,y) links (using computation of theta3)
    La = l2x
    Lb = l2y
    Lc = l3x - l2x
    Ld = l3y - l2y
    Le = (l4x - l3x)*np.cos(theta3)
    Lf = l4y - l3y
    L1 = np.sqrt(La**2 + Lb**2)
    L2 = np.sqrt((Lc + Le)**2 + (Ld+Lf)**2)

    # Obtain inverse kinematics
    # th2_nom = np.arccos((x**2 + y**2 - L1**2 - L2**2) / (2*L1*L2))
    # th1_nom = np.arctan2(y, x) - np.arctan2(L2*np.sin(th2_nom), L1 + L2*np.cos(th2_nom))
    th2_nom = -np.pi + np.arccos((-x**2 + -y**2 + L1**2 + L2**2) / (2*L1*L2))
    th1_nom = np.arctan2(y, x) + np.arccos((x**2 + y**2 + L1**2 - L2**2) / (2*L1*np.sqrt(x**2 + y**2)))
    
    th_b = np.arctan2(Ld + Lf, Lc + Le)
    th_a = np.arctan2(Lb, La)
    theta1_a = th1_nom - th_a
    theta2_a = th2_nom - th_b
    
    # Print out theta values
    # print("Theta 1: " + str(theta1_a) + " , Theta 2: " + str(theta2_a) + " , Theta 3: " + str(theta3))
    return (theta4, theta2_a, 0.0, theta1_a, theta3)
    # return (theta1_a, theta2_a, theta3, theta4)

def fkin_twist(joints):
    th1, th2, th3, th4 = joints[0], joints[1], joints[2], joints[3]
    g_base = np.array([[1, 0, 0, l4x], [0, 1, 0, l4y], [0, 0, 1, l4z], [0, 0, 0, 1]])
    g_1 = np.array([[np.cos(th1), -np.sin(th1), 0, 0], [np.sin(th1), np.cos(th1), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) 
    g_2 = np.array([[np.cos(th2), -np.sin(th2), 0, l2x*(1 - np.cos(th2)) + l2y*np.sin(th2)], [np.sin(th2), np.cos(th2), 0, -l2x*np.sin(th2) + l2y*(1 - np.cos(th2))], [0, 0, 1, 0], [0, 0, 0, 1]])
    g_3 = np.array([[np.cos(th3), 0, np.sin(th3), l3x*(1 - np.cos(th3)) - l3z*np.sin(th3)], [0, 1, 0, 0], [-np.sin(th3), 0, np.cos(th3), l3x*np.sin(th3) + l3z*(1 - np.cos(th3))], [0, 0, 0, 1]])
    g_4 = np.array([[np.cos(th4), 0, np.sin(th4), l4x*(1 - np.cos(th4)) - l4z*np.sin(th4)], [0, 1, 0, 0], [-np.sin(th4), 0, np.cos(th4), l4x*np.sin(th4) + l4z*(1 - np.cos(th4))], [0, 0, 0, 1]])
    g_out = np.matmul(np.matmul(np.matmul(np.matmul(g_1, g_2), g_3), g_4), g_base)
    R = g_out[0:3, 0:3]
    x = g_out[0:3, 3]
    return x

def fkin(joints):
    th1, th2, th3, th4 = joints[3], joints[1], joints[4], joints[0]

    La = l2x
    Lb = l2y
    Lc = l3x - l2x
    Ld = l3y - l2y
    Le = (l4x - l3x)*np.cos(th3)
    Lf = l4y - l3y
    L1 = np.sqrt(La**2 + Lb**2)
    L2 = np.sqrt((Lc + Le)**2 + (Ld+Lf)**2)

    th = th1 + th2
    phi = th3 + th4
    x2 = La*np.cos(th1) - Lb*np.sin(th1)
    y2 = La*np.sin(th1) + Lb*np.cos(th1)
    x4 = (Lc + Le)*np.cos(th1+th2) - (Ld + Lf)*np.sin(th1+th2)
    y4 = (Lc + Le)*np.sin(th1+th2) + (Ld + Lf)*np.cos(th1+th2)
    x = x2 + x4
    y = y2 + y4
    z = l4z - (l4x - l3x)*np.sin(th3)
    return (x, y, z, phi)

if __name__ == '__main__':
    joints = np.array([0.0, 1.57079632679, 0.0, 0.0])
    joints = np.array([0.92, 0.472, 0.372, 0.0])
    f = fkin_twist(joints)
    print(f)
    q = ikin(f)
    print(joints)
    print(q)

