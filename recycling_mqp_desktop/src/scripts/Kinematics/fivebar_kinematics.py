import numpy as np
import math

# should change this to l1 and l2 to match the ref doc
# http://www.mnrlab.com/uploads/7/3/8/3/73833313/modeling-of-pantograph.pdf
a1 = 111.28
a2 = 146.605
x = 35.25

# to be used to calculate the forward kinematics of the robotic arm
# NEED TO FORMAT TO FIT COMMENT CONVENTIONS
# unless otherwise specified, lengths are given in mm, force in N, torques in N-mm, and angles in radians

def fwkin(theta):

    # assume that the linkage is symmetric
    # for different servo angles, see systems of equations in the link on LINE 6
    ang1 = math.pi - theta
    ang2 = calc_q2(theta)
    y = a1*math.sin(ang1) + a2*math.sin(ang2)
    return [y, ang2]

def calc_q2(ang1):
    [xa, ya] = [a1*math.cos(ang1), a1*math.sin(ang1)]
    q2 = math.acos((xa + 35.25)/a2)
    return q2

def jacobian(theta):
    ang1 = math.pi - theta
    ang2 = calc_q2(theta)

    J = np.array([[-a1*math.sin(ang1), -a2*math.sin(ang2)],
                    [a1*math.cos(ang1), a2*math.cos(ang2)]])
    return J

def calc_torque_j1(theta, x_force, y_force):
    ang1 = math.pi - theta
    Q1 = -x_force*a1*math.sin(ang1) + y_force*a2*math.cos(ang1)
    return Q1

print(fwkin(0.5))
