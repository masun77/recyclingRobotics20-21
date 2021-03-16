#!/usr/bin/env python3
import time
import rospkg
import rospy
import serial.serialutil
from _cffi_backend import callback
# import roslib
# roslib.load_manifest('recycling_mqp_dynamixels')
# import sys
# sys.path.insert(0, "recycling_mqp_desktop")

from recycling_mqp_msgs.msg import *
from recycling_mqp_msgs.srv import *

# from recycling_mqp_msgs.msg.StepperStatus import x_aligned
# from recycling_mqp_msgs.msg import x_aligned
# from recycling_mqp_msgs.msg import x_aligned

# import dynamixel_sdk as dynamixel
import recycling_mqp_dynamixels.dynamixel_sdk as dynamixel
from recycling_mqp_desktop.src.scripts.dynamixel_control import DynamixelMotor
from recycling_mqp_desktop.src.scripts.dynamixel_control import config

# controller port
CONTROLLER_DEV = "/dev/ttyUSB0"

# motor ports
ARM_A_ID = 1
ARM_B_ID = 2
GRIPPER_ID = 3

# publishers
gripper_pub = None
arm_pub = None

# motors
arm_a = None
arm_b = None
gripper = None

# home location
ARM_A_HOME = 1300
ARM_B_HOME = 2800
GRIPPER_HOME = 2000

# pick up position
ARM_A_PICK = 1900
ARM_B_PICK = 2200
GRIPPER_PICK = 1360


# Connect to the motors if possible (otherwise exit) and enable torque
def initialize_motors():
    global arm_a, arm_b, gripper
    rospy.loginfo("Initializing Dynamixel motors...")

    # create port for U2D2
    port = dynamixel.PortHandler(CONTROLLER_DEV.encode('utf-8'))
    # port = dynamixel.PortHandler(CONTROLLER_DEV)

    # attempt to connect to U2D2
    try:
        port.openPort()
    except serial.serialutil.SerialException:
        rospy.logfatal("Failed to connect to U2D2 controller at %s" % CONTROLLER_DEV)
        exit(0)

    rospy.loginfo("Connected to U2D2")

    # initialize packet handler (protocol 2)
    packet_handler = dynamixel.PacketHandler(2)

    # arm and gripper motors
    arm_a = DynamixelMotor(ARM_A_ID, port, packet_handler)
    arm_b = DynamixelMotor(ARM_B_ID, port, packet_handler)
    gripper = DynamixelMotor(GRIPPER_ID, port, packet_handler)

    # enable torque
    arm_a.enable_torque()
    rospy.logwarn("Arm A dynamixel torque enabled")
    arm_b.enable_torque()
    rospy.logwarn("Arm B dynamixel torque enabled")
    gripper.enable_torque()
    rospy.logwarn("Gripper dynamixel torque enabled")

    return port


# Move the arm and gripper to home position
def home():
    global arm_a, arm_b, gripper
    arm_a.set_goal_position(ARM_A_HOME)
    arm_b.set_goal_position(ARM_B_HOME)
    gripper.set_goal_position(GRIPPER_HOME)


def pick_up():
    global arm_a, arm_b, gripper
    arm_a.set_goal_position(ARM_A_PICK)
    arm_b.set_goal_position(ARM_B_PICK)
    if not arm_a.is_moving():
        gripper.set_goal_position(GRIPPER_PICK)


def drop_off():
    global arm_a, arm_b, gripper
    arm_a.set_goal_position(ARM_A_HOME)
    arm_b.set_goal_position(ARM_B_HOME)


# Initialize motors and home arm/gripper.
# While ROS is running, publish arm position...
# def motor_control():
#     global arm_a, arm_b, gripper
#     rospy.init_node('dynamixelcontrol', anonymous=False)
#     port = initialize_motors()
#     rate = rospy.Rate(10)  # 10hz
#     try:
#         while not rospy.is_shutdown():  # todo: how positions correspond to space
#             # publish arm status
#             home()
#             arm_pub.publish(arm_a.get_position()[0], arm_b.get_position()[0], arm_a.is_moving()[0],
#                             arm_b.is_moving()[0])
#
#             # publish gripper status
#             gripper_pos = gripper.get_position()[0]
#             gripper_pub.publish(gripper_pos, config.GRIPPER_OPEN_POS == gripper_pos, gripper.is_moving()[0])
#
#             # todo: id, pickup, and drop off item
#             pick_up()
#             time.sleep(2)
#             drop_off()
#
#             rate.sleep()
#     except rospy.ROSInterruptException:
#         pass
#
#     # new line for aesthetics
#     print("")
#
#     # disable torques
#     rospy.logwarn("Stopping Dynamixel motors...")
#     arm_a.disable_torque()
#     arm_b.disable_torque()
#     gripper.disable_torque()
#
#     # close port
#     port.closePort()
#     rospy.logwarn("Motor control node safely stopped")


# def gripper_control(set_open):
# 	if set_open == 1:
# 		success = gripper.set_goal_position(config.GRIPPER_OPEN_POS)
# 	else:
# 		success = gripper.set_goal_position(config.GRIPPER_CLOSED_POS)
#
# 	return GripperControlResponse(int(success))


# def arm_control(position):
# 	success = arm_a.set_goal_position(position)
#
# 	# if the first didn't fail...
# 	if success:
# 		success = arm_b.set_goal_position(position)
#
# 	return ArmControlResponse(int(success))

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#
#
# def listener():
#     rospy.init_node('listener', anonymous=True)
#
#     rospy.Subscriber("chatter", x_aligned, callback)
#
#     rospy.spin()


# Main function. Set publishing and services information, then call motorControl to initialize and run motors
if __name__ == '__main__':
    try:
        # gripper_pub = rospy.Publisher('gripper', GripperStatus, queue_size=1)
        # arm_pub = rospy.Publisher('arm', ArmStatus, queue_size=1)
        # motor_control()
        # listener()
        # rospy.init_node('dynamixel_control', anonymous=False)


        initialize_motors()
        action = input("Please write home or pickup")
        if action == "home":
            print("Homing!")
            home()
        elif action == "pickup":
            print("Picking up!")
            pick_up()
        else:
            print("Incorrect input")

    # arm_control_srv = rospy.ServiceProxy('arm_controller', ArmControl, arm_control)
    # gripper_control_srv = rospy.ServiceProxy('gripper_controller', GripperControl, gripper_control)

    except rospy.ROSInterruptException:
        pass
