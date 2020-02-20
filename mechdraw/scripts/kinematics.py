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
lty = 0.00916 - 0.095     # Need to Measure
ltz = 0.19078


# TODO: Add workspace check
def ikin(tip):
    x, y, z, phi = tip[0], tip[1], tip[2], tip[3]
    # Compute theta3 to get desired z                                    
    Lz = np.sqrt((l4x - l3x)**2 + (l4z - l3z)**2)
    theta_z_base = np.arctan2(l4z - l3z, l4x - l3x)
    condz = -(z - l3z) / Lz 
    if (condz < -1 or condz > 1):
        print("Desired point outside of workspace")
        return None
    theta3_tot = np.arcsin( -(z - l3z) / Lz )
    theta3 = np.arcsin( -(z - l3z) / Lz) + theta_z_base

    # Compute important (x,y) links (using computation of theta3)   
    La = l2x
    Lb = l2y
    Lc = l3x - l2x
    Ld = l3y - l2y
    Le = Lz*np.cos(theta3_tot)
    Lf = lty - l3y
    L1 = np.sqrt(La**2 + Lb**2)
    L2 = np.sqrt((Lc + Le)**2 + (Ld + Lf)**2)

    # Obtain inverse kinematics
    cond1 = (-x**2 + -y**2 + L1**2 + L2**2) / (2*L1*L2)
    cond2 = (x**2 + y**2 + L1**2 - L2**2) / (2*L1*np.sqrt(x**2 + y**2))
    if (cond1 > 1 or cond1 < -1 or cond2 > 1 or cond2 < -1):
        print("Desired point outside of workspace")
        return None
    th2_nom = -np.pi + np.arccos((-x**2 + -y**2 + L1**2 + L2**2) / (2*L1*L2))
    th1_nom = np.arctan2(y, x) + np.arccos((x**2 + y**2 + L1**2 - L2**2) / (2*L1*np.sqrt(x**2 + y**2)))
    th_b = np.arctan2(Ld + Lf, Lc + Le)
    th_a = np.arctan2(Lb, La)
    theta1_a = th1_nom - th_a
    theta2_a = th2_nom - th_b + th_a

    # Set gripper orientation
    theta4 = phi - theta3
    
    return (theta1_a, theta2_a, theta3, theta4)


def fkin_twist(joints):
    th1, th2, th3, th4 = joints[0], joints[1], joints[2], joints[3]
    g_base = np.array([[1, 0, 0, ltx], [0, 1, 0, lty], [0, 0, 1, ltz], [0, 0, 0, 1]])
    g_1 = np.array([[np.cos(th1), -np.sin(th1), 0, 0], [np.sin(th1), np.cos(th1), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) 
    g_2 = np.array([[np.cos(th2), -np.sin(th2), 0, l2x*(1 - np.cos(th2)) + l2y*np.sin(th2)], [np.sin(th2), np.cos(th2), 0, -l2x*np.sin(th2) + l2y*(1 - np.cos(th2))], [0, 0, 1, 0], [0, 0, 0, 1]])
    g_3 = np.array([[np.cos(th3), 0, np.sin(th3), l3x*(1 - np.cos(th3)) - l3z*np.sin(th3)], [0, 1, 0, 0], [-np.sin(th3), 0, np.cos(th3), l3x*np.sin(th3) + l3z*(1 - np.cos(th3))], [0, 0, 0, 1]])
    g_4 = np.array([[np.cos(th4), 0, np.sin(th4), l4x*(1 - np.cos(th4)) - l4z*np.sin(th4)], [0, 1, 0, 0], [-np.sin(th4), 0, np.cos(th4), l4x*np.sin(th4) + l4z*(1 - np.cos(th4))], [0, 0, 0, 1]])
    g_out = np.matmul(np.matmul(np.matmul(np.matmul(g_1, g_2), g_3), g_4), g_base)
    R = g_out[0:3, 0:3]
    x = g_out[0:3, 3]
    phi, th = joints[2], joints[0] + joints[1]
    return (x[0], x[1], x[2], phi, th)

def fkin_check(joints):
    th1, th2, th3, th4 = joints[0], joints[1], joints[2], joints[3]
    theta_z_base = np.arctan2(l4z - l3z, l4x - l3x)
    Lz = np.sqrt((l4x - l3x)**2 + (l4z - l3z)**2)
    Le = Lz*np.cos(th3 - theta_z_base)
    x = ( l2x*np.cos(th1) - l2y*np.sin(th1) ) + ( (l3x - l2x)*np.cos(th1 + th2) - (l3y - l2y)*np.sin(th1 + th2) ) + Le*np.cos(th1 + th2) - (l4y - l3y)*np.sin(th1 + th2) - (lty - l4y)*np.sin(th1 + th2)
    y = ( l2x*np.sin(th1) + l2y*np.cos(th1) ) + ( (l3x - l2x)*np.sin(th1 + th2) + (l3y - l2y)*np.cos(th1 + th2) ) + Le*np.sin(th1 + th2) + (l4y - l3y)*np.cos(th1 + th2) + (lty - l4y)*np.cos(th1 + th2)
    return x, y

def fkin(joints):
    th1, th2, th3, th4 = joints[0], joints[1], joints[2], joints[3]

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
    return (x, y, z, phi, th)

def motor_pos(joints):
    th1, th2, th3, th4 = joints[0], joints[1], joints[2], joints[3]
    g_base2 = np.array([[1, 0, 0, l2x], [0, 1, 0, l2y], [0, 0, 1, l2z], [0, 0, 0, 1]])
    g_base3 = np.array([[1, 0, 0, l3x], [0, 1, 0, l3y], [0, 0, 1, l3z], [0, 0, 0, 1]])
    g_base4 = np.array([[1, 0, 0, l4x], [0, 1, 0, l4y], [0, 0, 1, l4z], [0, 0, 0, 1]])
    g_1 = np.array([[np.cos(th1), -np.sin(th1), 0, 0], [np.sin(th1), np.cos(th1), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    g_2 = np.array([[np.cos(th2), -np.sin(th2), 0, l2x*(1 - np.cos(th2)) + l2y*np.sin(th2)], [np.sin(th2), np.cos(th2), 0, -l2x*np.sin(th2) + l2y*(1 - np.cos(th2))], [0, 0, 1, 0], [0, 0, 0, 1]])
    g_3 = np.array([[np.cos(th3), 0, np.sin(th3), l3x*(1 - np.cos(th3)) - l3z*np.sin(th3)], [0, 1, 0, 0], [-np.sin(th3), 0, np.cos(th3), l3x*np.sin(th3) + l3z*(1 - np.cos(th3))], [0, 0, 0, 1]])
    g_4 = np.array([[np.cos(th4), 0, np.sin(th4), l4x*(1 - np.cos(th4)) - l4z*np.sin(th4)], [0, 1, 0, 0], [-np.sin(th4), 0, np.cos(th4), l4x*np.sin(th4) + l4z*(1 - np.cos(th4))], [0, 0, 0, 1]])
    g_out2 = np.matmul(g_1, g_base2)
    g_out3 = np.matmul(np.matmul(g_1, g_2), g_base3)
    g_out4 = np.matmul(np.matmul(np.matmul(g_1, g_2), g_3), g_base4)
    x2 = g_out2[0:3, 3]
    x3 = g_out3[0:3, 3]
    x4 = g_out4[0:3, 3]
    return x2, x3, x4


# TODO: Get Jacobian
def get_jacobian(joints):
    x2, x3, x4 = motor_pos(joints)
    x1 = np.array([0, 0, 0])
    w1 = np.array([0, 0, 1])
    w2 = np.array([0, 0, 1])
    w3 = np.array( [ -np.sin(joints[0] + joints[1]), np.cos(joints[0] + joints[1]), 0. ] )
    w4 = np.array( [ -np.sin(joints[0] + joints[1]), np.cos(joints[0] + joints[1]), 0. ] )
    J = np.stack( [ np.concatenate([np.zeros(3), w1]), np.concatenate([-np.cross(w2, x2), w2]), np.concatenate([-np.cross(w3, x3), w3]), np.concatenate([-np.cross(w4, x4), w4]) ] , axis=1 )
    return J

def get_iJacobian(joints):
    J_inv = np.linalg.pinv(get_jacobian(joints))
    return J_inv

if __name__ == '__main__':
    joints = np.array([0.92, 0.472, 0.372, 0.0])
    f = fkin_twist(joints)
    q = ikin(f)
    J = get_jacobian(q)
    J_inv = get_iJacobian(q)
    f_test = fkin_twist(q)
    f_check = fkin_check(q)
    print(f_test)
    print(f_check)
    q_test = ikin(f_test)

    q_out = ikin(np.array([0.6, 0.6, 0.4, 0.0]))


