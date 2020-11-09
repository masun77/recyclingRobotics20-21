#ifndef MQP_STEPPER_CONTROLLER_H_
#define MQP_STEPPER_CONTROLLER_H_

const int CONTROL_RUNNING = 2;
const int CONTROL_STOPPED = 1;
const int CONTROL_ESTOP = 0;
const int UPDATE_INTERVAL_MILLISECONDS = 200;

// defines the control state
typedef struct state {
	uint8_t control_state;
	uint8_t enabled;
	uint64_t comms_watchdog;
} state_t;

#endif
