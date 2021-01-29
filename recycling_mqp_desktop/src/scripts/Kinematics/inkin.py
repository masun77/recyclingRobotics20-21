import math
import numpy as np

# to be used to calculate the inverse kinematics of the robotic arm
a1 = 111.28
a2 = 146.605

# calculates the joint values (radians) given x and y coordinates (mm)
def inkin(x, y):
    num2 = a1**2 + a2**2 - (x**2 + y**2)
    den2 = 2*a1*a2
    theta2 = math.acos(-num2/den2)

    alpha = math.atan2(y,x)
    num1 = a1**2 + x**2 + y**2 - a2**2
    den1 = 2*a1*math.sqrt(x**2 + y**2)
    beta = math.acos(num1/den1)
    theta1 = (alpha - beta)

    return [theta1,theta2]

print(inkin(-10, 200))