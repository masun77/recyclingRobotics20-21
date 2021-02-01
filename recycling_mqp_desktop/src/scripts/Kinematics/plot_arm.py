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
    joint_y = a1*math.cos(math.pi - theta)
    plt.plot([0, joint_x], [0, joint_y,])
    plt.show()

    
# plot_robot(0.5)
x = np.linspace(0, 20, 100)  # Create a list of evenly-spaced numbers over the range
plt.plot(x, np.sin(x))       # Plot the sine of each x point
plt.show() 