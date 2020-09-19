#!/usr/bin/env python

import rospy
import serial
import struct

import stepper_service

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
def encode_packet(packet):
	packed = struct.pack("<B?BxiiiBBBx", packet)
	return packed

def checkXPos(x_a, x_b):
	if x_a != x_b:
		rospy.logerr_throttle(0.5, "X-axis motors are de-sync'd! \
							Off by %d steps" % abs(x_a - x_b))

def checkXDir(x_a_dir, x_b_dir):
	if x_a_dir != x_b_dir:
		rospy.logerr_throttle(0.5, "X-axis motor directions are not the same? \
							A is %d and B is %d" % (x_a_dir, x_b_dir))

def getStatus(statusBit):
	statusIndex = (statusBit > 2 and 3) or statusBit
	status = possibleStatuses[statusIndex]
	if (statusIndex == 3):
		rospy.logerr_throttle_identical(1, "Failed to decode status from Teensy (%d)" % packet_data[0])
	return status

def checkStatusesMakeSense(limit_status):
	if limit_status["x_min"] and limit_status["x_max"]:
		rospy.logwarn_throttle(1, "Both X minimum and X maximum limits are triggered, check system.")
	elif limit_status["z_min"] and limit_status["z_max"]:
		rospy.logwarn_throttle(1, "Both Z minimum and Z maximum limits are triggered, check system.")

def getLimitStatuses(packetInfo):
	limits = [False, False, False, False, False, False, False, False]
	for i in range(0, 8):
		limits[i] = (packetInfo & (1 << i)) > 0
	limit_status = {
		"x_min": limits[0] or limits[1],
		"x_max": limits[2] or limits[3],
		"z_min": limits[4] or limits[5],
		"z_max": limits[6] or limits[7]
	}
	checkStatusesMakeSense(limit_status)
	return limit_status

# todo comments
def decode_packet(packet):
	# Get the data as a tuple of values unpacked according to the format string (first argument)
	# B is ... x is butter... TODO
	packet_data = struct.unpack("<B?BxiiiBBBx", packet)

	status = getStatus(packet_data[0])
	limit_status = getLimitStatuses(packet_data[2])
	x_a = packet_data[3]
	x_b = packet_data[4]
	checkXPos(x_a, x_b)
	x_a_dir = packet_data[6]
	x_b_dir = packet_data[7]
	checkXDir(x_a_dir, x_b_dir)
	# todo: might want to do more checks than this. e.g. why don't they check z?

	return {
		"control_status": status,
		"control_status_byte": packet_data[0],
		"enabled": packet_data[1],
		"limits": limit_status,
		"aligned": x_a == x_b,
		"x": x_a,
		"x_dir": x_a_dir,
		"z": packet_data[5],
		"z_dir": packet_data[8]
	}


def safe_exit():
	rospy.loginfo("Control state changed to Stopped (Disconnected)")

	control_state[0] = 1
	control_state[1] = "Stopped (Disconnected)"

	control_pub.publish(control_state[0], control_state[1])
	exit(0)

def setPublishers():
	stepper_pub = rospy.Publisher('steppers', StepperStatus, queue_size=1)
	limit_pub = rospy.Publisher('limit_switches', LimitStatus, queue_size=1)
	control_pub = rospy.Publisher('system_control_state', ControlStatus, queue_size=1)
	return stepper_pub, limit_pub, control_pub

def connectToUSB():
	usb = None
	try:
		usb = serial.Serial(TEENSY_PORT, 115200)
	except serial.serialutil.SerialException:
		rospy.logfatal("Failed to open serial connection to Teensy at %s" % TEENSY_PORT)
		safe_exit()
	rospy.loginfo("Connected to Teensy")
	return usb

def checkNoDroppedBits(info):
	if info != "<status>":
		# TODO recover from dropped bytes?
		rospy.logfatal("Serial stream dropped bytes!")
		safe_exit()

def publishData(stepper_pub, limit_pub, data):
	stepper_pub.publish(data["enabled"], data["aligned"], data["x"], data["z"], data["x_dir"],
						data["z_dir"])
	limit_pub.publish(data["limits"]["x_min"], data["limits"]["x_max"], data["limits"]["z_min"],
								  data["limits"]["z_max"])

def setAndPublishControlStatus(controlStatusByte, status):
	if controlStatusByte != control_state[0]:
		rospy.loginfo("Control state changed to %s" % status)

	# record control state
	control_state[0] = controlStatusByte
	control_state[1] = status
	control_pub.publish(control_state[0], control_state[1])

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
