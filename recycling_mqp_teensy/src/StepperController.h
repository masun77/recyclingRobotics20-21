#ifndef MQP_STEPPER_CONTROLLER_H_
#define MQP_STEPPER_CONTROLLER_H_

#define CONTROL_RUNNING	2
#define CONTROL_STOPPED	1
#define CONTROL_ESTOP	0
#define UPDATE_INTERVAL_MILLISECONDS 200;

typedef struct state {
	uint8_t control_state;
	uint8_t enabled;
	uint64_t comms_watchdog;
} state_t;

#endif
