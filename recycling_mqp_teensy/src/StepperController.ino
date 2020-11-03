#include <inttypes.h>
#include <MultiStepper.h>    // Allows control of multiple steppers at once
#include <AccelStepper.h>

#include "StepperController.h"
#include "pins.h"
#include "comms.h"

state_t state;
packet_send_t packet;
unsigned long prev_millis = 0;
int updateIntervalMilliseconds = 200;

MultiStepper stepper_x;
AccelStepper stepper_x1(AccelStepper::DRIVER, STR3_X1_STEP, STR3_X1_DIR, 0, 0, false);
AccelStepper stepper_x2(AccelStepper::DRIVER, STR3_X2_STEP, STR3_X2_DIR, 0, 0, false);
AccelStepper stepper_z(AccelStepper::DRIVER, STR3_Z_STEP, STR3_Z_DIR, 0, 0, false);

// TODO comments
void setStepperEnablePins() {
	stepper_x1.setEnablePin(STR3_X1_EN);
	stepper_x2.setEnablePin(STR3_X2_EN);
	stepper_z.setEnablePin(STR3_Z_EN);
}

void invertPins() {
	stepper_x1.setPinsInverted(true, true, true);
	stepper_x2.setPinsInverted(true, true, true);
	stepper_z.setPinsInverted(true, true, true);
}

void createMultiStepperX() {
	stepper_x.addStepper(stepper_x1);
	stepper_x.addStepper(stepper_x2);
}

void setZstepper() {
	stepper_z.setMaxSpeed(2000);
	stepper_z.setAcceleration(1400.0);
	stepper_z.enableOutputs();
}

void setLimitSwitches() {
	pinMode(LIM_X_MIN_A, INPUT_PULLUP);
	pinMode(LIM_X_MIN_B, INPUT_PULLUP);
	pinMode(LIM_X_MAX_A, INPUT_PULLUP);
	pinMode(LIM_X_MAX_B, INPUT_PULLUP);

	pinMode(LIM_Z_MIN_A, INPUT_PULLUP);
	pinMode(LIM_Z_MIN_B, INPUT_PULLUP);
	pinMode(LIM_Z_MAX_A, INPUT_PULLUP);
	pinMode(LIM_Z_MAX_B, INPUT_PULLUP);
}

void blinkLight() {    // TODO this vs blink2?
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);
	delay(1000);
}

void blinkLight2() {
	digitalWrite(LED_PIN, LOW);
	delay(2000);
	digitalWrite(LED_PIN, HIGH);
}

void setState() {
	state.control_state = CONTROL_STOPPED;
	state.enabled = false;
	state.comms_watchdog = millis();
}

int8_t speed_to_direction(float speed) {
	if (speed > 0) {
		return 1;
	} else if (speed < 0) {
		return -1;
	}
	return 0;
}

/*
When you power on the board or press reset, this function runs once.
*/
void setup() {
    setStepperEnablePins();
    invertPins();
    createMultiStepperX();
    setZstepper();
    setLimitSwitches();
    blinkLight();
    setState();
	Serial.begin(115200);
    set_status_packet(&packet);
    blinkLight2();

	stepper_z.move(4000);    // TODO why have this?
}

void set_status_packet(packet_send_t* packet) {
	packet->control_state = state.control_state;  // todo: how can this get changed?
	packet->enabled = state.enabled;
	packet->limit = 0x0;    // todo: update based on limit_switches?
	packet->stepper_positions[0] = stepper_x1.currentPosition();
	packet->stepper_positions[1] = stepper_x2.currentPosition();
	packet->stepper_positions[2] = stepper_z.currentPosition();
	packet->stepper_directions[0] = speed_to_direction(stepper_x1.speed());
	packet->stepper_directions[1] = speed_to_direction(stepper_x2.speed());
	packet->stepper_directions[2] = speed_to_direction(stepper_z.speed());
}

void switchLight() {
	digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

/**
 * Main loop. Runs repeatedly while the board is on.
 * TODO: receive and execute input
*/
void loop() {
	if (millis() - prev_millis > updateIntervalMilliseconds) {
	    switchLight();
		prev_millis = millis();
		set_status_packet(&packet);
		send_packet(&packet);
	}

	// update stepper outputs todo
	stepper_z.run();
}

