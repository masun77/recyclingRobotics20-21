#!/usr/bin/env python

import rospy
import serial
import struct

import stepper_service

from recycling_mqp_messages.msg import *
from recycling_mqp_messages.msg import StepperStatus, LimitStatus, ControlStatus
from recycling_mqp_messages.srv import StepperControl

# controller port
TEENSY_PORT = '/dev/ttyACM0'

# publishers
control_pub = None

# state
control_state = [-1, "Initializing"]

possibleStatuses = ["Emergency Stopped", "Stopped", "Running", "Unknown"]

# TODO
# @param packet: what does the Teensy need to know from master?
# @return packed: the packet with the information to be sent to the master
def encode_packet(packet):
	packed = struct.pack("<B?BxiiiBBBx", packet)
	return packed

# @param x_a: the current position of the arm on the x axis
# @param x_b: the desired position of the arm on the x axis
# @return: informs user how off the position is to the desired one
def checkXPosition(x_a, x_b):
	if x_a != x_b:
		rospy.logerr_throttle(0.5, "X-axis motors are de-sync'd! \
							Off by %d steps" % abs(x_a - x_b))

# @param x_a_dir: the current direction of the arm on the x axis
# @param x_b_dir: the desired direction of the arm on the x axis
# @return: informs user how off the current direction is to the desired one
def checkXDirection(x_a_dir, x_b_dir):
	if x_a_dir != x_b_dir:
		rospy.logerr_throttle(0.5, "X-axis motor directions are not the same? \
							A is %d and B is %d" % (x_a_dir, x_b_dir))

# @param statusBit: received status from the master
# @return status: tells user if the status is okay or if the decode has failed
def getStatus(statusBit):
	statusIndex = (statusBit > 2 and 3) or statusBit
	status = possibleStatuses[statusIndex]
	if (statusIndex == 3):
		rospy.logerr_throttle_identical(1, "Failed to decode status from Teensy (%d)" % packet_data[0])
	return status

# @param limit_status: gives the limit for the x and z position
# @return: tells if the arm reaches the limits
def checkStatusesMakeSense(limit_status):
	if limit_status["x_min"] and limit_status["x_max"]:
		rospy.logwarn_throttle(1, "Both X minimum and X maximum limits are triggered, check system.")
	elif limit_status["y_min"] and limit_status["y_max"]:
		rospy.logwarn_throttle(1, "Both Y minimum and Y maximum limits are triggered, check system.")

# @param packetInfo: contains the info form the packet
# @return limit_status: returns the status of the limit reached
def getLimitStatuses(packetInfo):
	limits = [False, False, False, False, False, False, False, False]
	for i in range(0, 8):
		limits[i] = (packetInfo & (1 << i)) > 0
	limit_status = {
		"x_min": limits[0] or limits[1],    # todo: verify that x and y correspond to physical x and y
		"x_max": limits[2] or limits[3],
		"y_min": limits[4] or limits[5],
		"y_max": limits[6] or limits[7]
	}
	checkStatusesMakeSense(limit_status)
	return limit_status

# todo comments
# @param packet: the information retrieved from the master
# @return: tells the user all the information retrieved from the packet
def decode_packet(packet):
	# Get the data as a tuple of values unpacked according to the format string (first argument)
	# B is ... x is butter... TODO
	packet_data = struct.unpack("<B?BxiiiBBBx", packet)

	status = getStatus(packet_data[0])
	limit_status = getLimitStatuses(packet_data[2])
	x_a = packet_data[3]
	x_b = packet_data[4]
	checkXPosition(x_a, x_b)
	x_a_dir = packet_data[6]
	x_b_dir = packet_data[7]
	checkXDirection(x_a_dir, x_b_dir)
	# todo: might want to do more checks than this. e.g. why don't they check y?

	return {
		"control_status": status,
		"control_status_byte": packet_data[0],
		"enabled": packet_data[1],
		"limits": limit_status,
		"aligned": x_a == x_b,
		"x": x_a,
		"x_dir": x_a_dir,
		"y": packet_data[5],
		"y_dir": packet_data[8]
	}

# @return: stops the arm if anything has disconnected
def safe_exit():
	rospy.loginfo("Control state changed to Stopped (Disconnected)")

	control_state[0] = 1
	control_state[1] = "Stopped (Disconnected)"

	control_pub.publish(control_state[0], control_state[1])
	exit(0)

# @return stepper_pub:
# @return limit_pub:
# @return control_pub:
def setPublishers():
	stepper_pub = rospy.Publisher('steppers', StepperStatus, queue_size=1)
	limit_pub = rospy.Publisher('limit_switches', LimitStatus, queue_size=1)
	control_pub = rospy.Publisher('system_control_state', ControlStatus, queue_size=1)
	return stepper_pub, limit_pub, control_pub

# @return: connects to the usb
def connectToUSB():
	usb = None
	try:
		usb = serial.Serial(TEENSY_PORT, 115200)
	except serial.serialutil.SerialException:
		rospy.logfatal("Failed to open serial connection to Teensy at %s" % TEENSY_PORT)
		safe_exit()
	rospy.loginfo("Connected to Teensy")
	return usb

# @param info: gives info of the status
# @return: stops safely if any bytes are dropped
def checkNoDroppedBits(info):
	if info != "<status>":
		# TODO recover from dropped bytes?
		rospy.logfatal("Serial stream dropped bytes!")
		safe_exit()

# @param stepper_pub:
# @param limit_pub:
# @param data: the info available for the arm (position, aligned, etc.)
# @return: the data for the stepper and the limit
def publishData(stepper_pub, limit_pub, data):
	stepper_pub.publish(data["enabled"], data["aligned"], data["x"], data["y"], data["x_dir"],
						data["y_dir"])
	limit_pub.publish(data["limits"]["x_min"], data["limits"]["x_max"], data["limits"]["y_min"],
								  data["limits"]["y_max"])

# @param controlStatusByte: shows if the status byte is okay or not
# @param status: shows status of robot
# @return: sets the control status for the robot
def setAndPublishControlStatus(controlStatusByte, status, control_pub):
	if controlStatusByte != control_state[0]:
		rospy.loginfo("Control state changed to %s" % status)

	# record control state
	control_state[0] = controlStatusByte
	control_state[1] = status
	control_pub.publish(control_state[0], control_state[1])

# @param stepper_pub:
# @param limit_pub:
# @param control_pub:
# @return: the data for the stepper and the limit
def mainLoop(stepper_pub, limit_pub, control_pub):
	while not rospy.is_shutdown():
		try:
			if usb.in_waiting > 28:
				checkNoDroppedBits(usb.read(8))
				data = decode_packet(usb.read(20))
				publishData(stepper_pub, limit_pub, data)
				setAndPublishControlStatus(data["control_status_byte"], data["control_status"], control_pub)
				rospy.sleep(0.095)
		except IOError:
			safe_exit()
	print("")
	rospy.logwarn("Terminated communication with Teensy, setting control state to stopped")
	safe_exit()

if __name__ == '__main__':
	try:
		stepper_pub, limit_pub, control_pub = setPublishers()
		rospy.init_node('teensy_comms', anonymous=False)
		stepper_control_srv = rospy.ServiceProxy('stepper_controller', StepperControl, stepper_service.stepper_control)
		usb = connectToUSB()
		mainLoop(stepper_pub, limit_pub, control_pub)
	except rospy.ROSInterruptException:
		rospy.logfatal("Teensy communication interrupted")
		safe_exit()
		pass # todo what is this except doing?
