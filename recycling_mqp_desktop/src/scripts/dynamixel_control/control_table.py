

# control table for xc430-w150
# http://emanual.robotis.com/docs/en/dxl/x/xc430-w150/#specifications
class XC530W150:
	def __init__(self): pass

	class Limits:
		def __init__(self): pass

		ADDR_TEMP = 31				# 1 byte
		ADDR_MAX_VOLTAGE = 32		# 2 byte
		ADDR_MIN_VOLTAGE = 34		# 2 byte
		ADDR_PWM = 36				# 2 byte
		ADDR_VELOCITY = 44			# 4 byte
		ADDR_MAX_POS = 48			# 4 byte
		ADDR_MIN_POS = 52			# 4 byte

	class System:
		def __init__(self): pass

		ADDR_SHUTDOWN = 63			# 1 byte
		ADDR_TORQUE_ENABLE = 64		# 1 byte
		ADDR_LED = 65				# 1 byte

	class Error:
		def __init__(self): pass

		ADDR_HW_ERROR = 70			# 1 byte

	class Control:
		def __init__(self): pass

		ADDR_MOV_THRESHOLD = 24		# 4 byte

		ADDR_VEL_I_GAIN = 76		# 2 byte
		ADDR_VEL_P_GAIN = 78		# 2 byte
		ADDR_POS_D_GAIN = 80		# 2 byte
		ADDR_POS_I_GAIN = 82		# 2 byte
		ADDR_POS_P_GAIN = 84		# 2 byte
		ADDR_FEEDFORWARD_1 = 88		# 2 byte
		ADDR_FEEDFORWARD_2 = 90		# 2 byte

		ADDR_GOAL_PWM = 100			# 2 byte
		ADDR_GOAL_VEL = 104			# 4 byte
		ADDR_GOAL_POS = 116			# 4 byte

	class Status:
		def __init__(self): pass

		ADDR_MOVING = 122			# 1 byte
		ADDR_PRESENT_PWM = 124		# 2 byte
		ADDR_PRESENT_LOAD = 126		# 2 byte
		ADDR_PRESENT_VEL = 128		# 4 byte
		ADDR_PRESENT_POS = 132		# 4 byte

		ADDR_PRESENT_TEMP = 146		# 1 byte
