#ifndef MQP_STEPPER_CONTROLLER_H_
#define MQP_STEPPER_CONTROLLER_H_

const int CONTROL_RUNNING = 2;
const int CONTROL_STOPPED = 1;
const int CONTROL_ESTOP = 0;

const int UPDATE_INTERVAL_MILLISECONDS = 200;

const float MAX_SPEED = 1000;    // todo - verify constants
const float MIN_SPEED = 25;    // todo: does this affect going in the negative direction?
const int ACCELERATION = 500;
const int X_MAX_POS = 1500;
const int X_MIN_POS = 0;
const int Y_MAX_POS = 1500;
const int Y_MIN_POS = 0;

// defines the control state
typedef struct state {
	uint8_t control_state;
	uint8_t enabled;
	uint64_t comms_watchdog;
} state_t;

#endif
