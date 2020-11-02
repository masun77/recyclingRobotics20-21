
import rospy

from dynamixel_sdk import robotis_def
from control_table import XC530W150

class DynamixelMotor:
	# motor attributes
	id = None
	port = None
	pkt = None

	# runtime data
	goal_position = None

	# constants specific to our motors
	MIN_POS = 0
	MAX_POS = 4095
	PROTOCOL = 2
	BAUD_RATE = 1000000

	def __init__(self, motor_id, port, packet_handler):
		self.id = motor_id
		self.port = port
		self.pkt = packet_handler

	# attempt to enable torque
	def enable_torque(self):
		if self._write_1byte(XC530W150.System.ADDR_TORQUE_ENABLE, 1):
			rospy.loginfo("dynamixel #%d > torque enabled" % self.id)
			return True
		else:
			return False

	# attempt to disable torque
	def disable_torque(self):
		if self._write_1byte(XC530W150.System.ADDR_TORQUE_ENABLE, 0):
			rospy.loginfo("dynamixel #%d > torque disabled" % self.id)
			return True
		else:
			return False

	# attempt to enable the LED
	def enable_led(self):
		if self._write_1byte(XC530W150.System.ADDR_LED, 1):
			rospy.loginfo("dynamixel #%d > LED enabled" % self.id)
			return True
		else:
			return False

	# attempt to disable the LED
	def disable_led(self):
		if self._write_1byte(XC530W150.System.ADDR_LED, 0):
			rospy.loginfo("dynamixel #%d > LED disabled" % self.id)
			return True
		else:
			return False

	# check if it is moving
	def is_moving(self):
		moving, error = self._read_1byte(XC530W150.Status.ADDR_MOVING)
		return moving, error

	# get current motor position
	def get_position(self):
		position, error = self._read_4byte(XC530W150.Status.ADDR_PRESENT_POS)

		# dont worry about it...
		if position == 4294967295:
			rospy.logwarn("dynamixel #%d > position corrected from %d to 0" % (self.id, position))
			position = 0

		return position, error

	# set the goal position (will be limited to the min/max position)
	def set_goal_position(self, position):
		if position < self.MIN_POS:
			self.goal_position = self.MIN_POS
		elif position > self.MAX_POS:
			self.goal_position = self.MAX_POS
		else:
			self.goal_position = position

		if self._write_4byte(XC530W150.Control.ADDR_GOAL_POS, position):
			rospy.logdebug("dynamixel #%d > position set to %d" % (self.id, position))
			return True
		else:
			return False

	# get the goal position
	def get_goal_position(self):
		return self.goal_position

	# read one byte
	def _read_1byte(self, address):
		data, result, error = self.pkt.read1ByteTxRx(self.port, self.id, address)
		return data, self._io_handle_result(result, error)

	# write one byte
	def _write_1byte(self, address, byte):
		result, error = self.pkt.write1ByteTxRx(self.port, self.id, address, byte)
		return self._io_handle_result(result, error)

	# read four bytes
	def _read_4byte(self, address):
		data, result, error = self.pkt.read4ByteTxRx(self.port, self.id, address)
		return data, self._io_handle_result(result, error)

	# write four bytes
	def _write_4byte(self, address, data):
		result, error = self.pkt.write4ByteTxRx(self.port, self.id, address, data)
		return self._io_handle_result(result, error)

	def _io_handle_result(self, result, error):
		if result != robotis_def.COMM_SUCCESS:
			rospy.logerr(("dynamixel #%d > " % self.id) + self.pkt.getTxRxResult(result))
		elif error != 0:
			rospy.logerr(("dynamixel #%d > " % self.id) + self.pkt.getRxPacketError(error))
		else:
			return True

		# if did not succeed, return false
		return False
