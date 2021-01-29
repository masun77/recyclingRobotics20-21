import numpy as np
import scipy.linalg as la
import math

# should change this to l1 and l2 to match the ref doc
# http://www.mnrlab.com/uploads/7/3/8/3/73833313/modeling-of-pantograph.pdf
a1 = 111.28
a2 = 146.605

# to be used to calculate the forward kinematics of the robotic arm
# NEED TO FORMAT TO FIT COMMENT CONVENTIONS

# creates the homogeneous transformation matrix using TDH parameters
# d and a are in mm
# theta and alpha are in radians
def tdh(theta, d, a, alpha):
    tdh_matrix = np.array([[math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
                          [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
                          [0, math.sin(alpha), math.cos(alpha), d],
                          [0, 0, 0, 1]])
    return tdh_matrix

# transformation matrix from the base to the end effector
def fwkin_matrix(j1, j2):
    T_01 = tdh(j1, 0, a1, 0)
    T_12 = tdh(j2, 0, a2, 0)
    T_02 = T_01 @ T_12
    return T_02

# position of the end effector (mm)
def fwkin(j1, j2):
    matrix = fwkin_matrix(j1, j2)
    return [matrix[0,3], matrix[1,3]]

def fwkin_5bar(theta):

    # assume that the linkage is symmetric
    ang1 = math.pi - theta
    ang2 = calc_q2(theta)
    x = a1*math.cos(ang1) + a2*math.cos(ang2)
    y = a1*math.sin(ang1) + a2*math.sin(ang2)
    return [x,y]

def calc_q2(ang1):
    [xa, ya] = [a1*math.cos(ang1), a1*math.sin(ang1)]
    q2 = math.acos((xa + 35.25)/a2)
    return q2

def jacobian_5bar(theta):
    ang1 = math.pi - theta
    ang2 = calc_q2(theta)

    J = np.array([[-a1*math.sin(ang1), -a2*math.sin(ang2)],
                    [a1*math.cos(ang1), a2*math.cos(ang2)]])
    return J

def calc_torques_5bar_j1(theta, x_force, y_force):
    ang1 = math.pi - theta
    Q1 = -x_force*a1*math.sin(ang1) + y_force*a2*math.cos(ang1)
    return Q1

print(calc_torques_5bar_j1(0*math.pi, 0, 9.8*0.300))