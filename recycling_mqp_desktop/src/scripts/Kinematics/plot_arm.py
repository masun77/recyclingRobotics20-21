import numpy as np
import math
import matplotlib.pyplot as plt
import fivebar_kinematics as kin

x_end = 35.25
a1 = 111.28
a2 = 146.605

def plot_robot(theta):
    [y_end, ang2] = kin.fwkin(theta)
    joint_x = a1*math.cos(math.pi - theta)
    joint_y = a1*math.sin(math.pi - theta)
    side2_joint_x = -joint_x + 2*x_end

    plt.plot([0, joint_x], [0, joint_y])
    plt.plot([joint_x, x_end], [joint_y, y_end], 'r')
    plt.plot([2*x_end, side2_joint_x], [0, joint_y])
    plt.plot([side2_joint_x, x_end], [joint_y, y_end])
    plt.scatter([0, joint_x, x_end, 2*x_end, side2_joint_x], [0, joint_y, y_end, 0, joint_y], color='black')
    print([0, joint_x, x_end], [0, joint_y, y_end])
    plt.show()

    
plot_robot(0.5)