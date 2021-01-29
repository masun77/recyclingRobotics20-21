import numpy as np
import math
import inkin as ik

# to be used to calculate the inverse kinematics of the robotic arm
# useful for trajectory planning, forces, etc.
a1 = 111.28
a2 = 146.605

# creates the jacobian for the robot based on the 2 joint angles
def jacobian(j1, j2):
    J = np.array([[-a1*math.sin(j1) - a2*math.sin(j1+j2), -a2*math.sin(j1+j2)],
                    [a1*math.cos(j1) + a2*math.cos(j1+j2), a2*math.cos(j1+j2)],
                    [0, 0],
                    [0, 0],
                    [0, 0],
                    [1, 1]
    ])
    return J

# calculates the torque on the robot given 2 joint angles (rad) 
# and the force x and y components
def forces_on_robot(j1, j2, x_force, y_force):
    F_tip = np.array([x_force, y_force]).T
    J = jacobian(j1, j2)
    J = J[0:2, 0:2]
    T = J.T @ F_tip
    return T * 0.001


joints = ik.inkin(-10, 200)
print(forces_on_robot(joints[0], joints[1], 0, 3.75))

