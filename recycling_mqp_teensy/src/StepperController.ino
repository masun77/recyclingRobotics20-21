#include <inttypes.h>
#include <MultiStepper.h>    // Allows control of multiple steppers at once
#include <AccelStepper.h>    // Import into Arduino IDE

#include "StepperController.h"
#include "pins.h"
#include "comms.h"

state_t state;
packet_send_t packet;
unsigned long prev_millis = 0;
float MAX_SPEED = 60000;
float MIN_SPEED = 10000;
bool receivedXPosition = true;
bool receivedYPosition = true;

int X_MAX_POS = 5000; // todo
int X_MIN_POS = 0;
int Y_MAX_POS = 5000;
int Y_MIN_POS = 0;

MultiStepper multistepper_y;
AccelStepper stepper_y1(AccelStepper::DRIVER, STR3_Y1_STEP, STR3_Y1_DIR, 0, 0, false);
AccelStepper stepper_y2(AccelStepper::DRIVER, STR3_Y2_STEP, STR3_Y2_DIR, 0, 0, false);
AccelStepper stepper_x(AccelStepper::DRIVER, STR3_X_STEP, STR3_X_DIR, 0, 0, false);


/* Set the enable pin for each stepper motor.
*/
void setStepperEnablePins() {
	stepper_y1.setEnablePin(STR3_Y1_EN);
	stepper_y2.setEnablePin(STR3_Y2_EN);
	stepper_x.setEnablePin(STR3_X_EN);
}

/* Invert the pins for each stpper
*/
void invertPins() {
	stepper_y1.setPinsInverted(true, true, true);
	stepper_y2.setPinsInverted(true, true, true);
	stepper_x.setPinsInverted(true, true, true);
}

// Create the combined stepper of the two Y-axis motors
void createMultiStepperY() {
	multistepper_y.addStepper(stepper_y1);
	multistepper_y.addStepper(stepper_y2);
}

// Set the speed and acceleration of the Y multistepper
void setYstepper() {
	multistepper_y.setMaxSpeed(2000);
	multistepper_y.setAcceleration(1400.0);
	multistepper_y.enableOutputs();
}

// Set the limit switches to receive input on the appropriate pins
void setLimitSwitches() {
	pinMode(LIM_X_MIN_A, INPUT_PULLUP);
	pinMode(LIM_X_MIN_B, INPUT_PULLUP);
	pinMode(LIM_X_MAX_A, INPUT_PULLUP);
	pinMode(LIM_X_MAX_B, INPUT_PULLUP);

	pinMode(LIM_Y_MIN_A, INPUT_PULLUP);
	pinMode(LIM_Y_MIN_B, INPUT_PULLUP);
	pinMode(LIM_Y_MAX_A, INPUT_PULLUP);
	pinMode(LIM_Y_MAX_B, INPUT_PULLUP);
}

// Attach interrupt function to each limit switch pin
void setLimitSwitchInterrupts() {
	attachInterrupt(digitalPinToInterrupt(LIM_X_MIN_A), limXMinAInterrupt, CHANGE);
	attachInterrupt(digitalPinToInterrupt(LIM_X_MIN_B), limXMinBInterrupt, CHANGE);
	attachInterrupt(digitalPinToInterrupt(LIM_X_MAX_A), limXMaxAInterrupt, CHANGE);
	attachInterrupt(digitalPinToInterrupt(LIM_X_MAX_B), limXMaxBInterrupt, CHANGE);
	attachInterrupt(digitalPinToInterrupt(LIM_Y_MIN_A), limYMinAInterrupt, CHANGE);
	attachInterrupt(digitalPinToInterrupt(LIM_Y_MIN_B), limYMinBInterrupt, CHANGE);
	attachInterrupt(digitalPinToInterrupt(LIM_Y_MAX_A), limYMaxAInterrupt, CHANGE);
	attachInterrupt(digitalPinToInterrupt(LIM_Y_MAX_B), limYMaxBInterrupt, CHANGE);
}

void limXMinAInterrupt() {
    packet->limit = limit | B1;
}

void limXMinBInterrupt() {
    packet->limit = limit | B10;
}

void limXMaxAInterrupt() {
    packet->limit = limit | B100;
}

void limXMaxBInterrupt() {
    packet->limit = limit | B1000;
}

void limYMinAInterrupt() {
    packet->limit = limit | B10000;
}

void limYMinBInterrupt() {
    packet->limit = limit | B100000;
}

void limYMaxAInterrupt() {
    packet->limit = limit | B1000000;
}

void limYMaxBInterrupt() {
    packet->limit = limit | B10000000;
}

// Blink the LED
void blinkLight() {
	digitalWrite(LED_PIN, LOW);
	delay(2000);
	digitalWrite(LED_PIN, HIGH);
}

/* Set the state variable
    @param control the control state (see Steppercontroller.h)
    @param enabled true if the steppers should be enabled, false otherwise
*/
void setState(int control, bool enabled) {
	state.control_state = control;
	state.enabled = enabled;
	state.comms_watchdog = millis();
}

/*  Converts a speed to a direction +1 or -1
    @param speed the speed of the motor
    @return 1 if speed > 0; -1 if speed < 0; 0 otherwise
*/
int8_t speed_to_direction(float speed) {
	if (speed > 0) {
		return 1;
	} else if (speed < 0) {
		return -1;
	}
	return 0;
}

// Switch the light to off if currently on; on if currently off.
void switchLight() {
	digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

/*
When you power on the board or press reset, this function runs once.
Sets up the pins, steppers, limit switches, and state. Begins Serial connection.
*/
void setup() {
    setStepperEnablePins();
    invertPins();
    createMultiStepperY();
    setYstepper();
    setLimitSwitches();
	pinMode(LED_PIN, OUTPUT);
    blinkLight();
    setState(CONTROL_STOPPED, false);
	Serial.begin(115200);
    set_status_packet();
	packet->limit = 0x0;  // Initial limit switch values all 0
    blinkLight();

    // todo: clean this up so there's a builder function for setup/easy to switch between setup strategies
	home();
	setSpeedManually();
}

// Send steppers to limit switches to set their position
void home() {
    stepper_x.setMaxSpeed(MIN_SPEED);
    stepper_x.move(-10000);   // todo: how far/direction to move? also, need to stop motors on interrupt?
    stepper_x.runToPosition();
    // stepper_x.setCurrentPosition(0);

    multistepper_y.setMaxSpeed(MIN_SPEED);
    multistepper_y.move(-10000);   // todo: how far/direction to move? also, need to stop motors on interrupt?
    multistepper_y.runToPosition();
    // multistepper_y.setCurrentPosition(0);
}

/* Set the status packet to send to the main robot controller.
    @ packet the address of the packet to send
*/
void set_status_packet() {
	packet->control_state = state.control_state;  // todo: how can state get changed?
	packet->enabled = state.enabled;  // todo: do something if a limit switch is triggered?
	packet->stepper_positions[0] = stepper_y1.currentPosition();
	packet->stepper_positions[1] = stepper_y2.currentPosition();
	packet->stepper_positions[2] = stepper_x.currentPosition();
	packet->stepper_directions[0] = speed_to_direction(stepper_y1.speed());
	packet->stepper_directions[1] = speed_to_direction(stepper_y2.speed());
	packet->stepper_directions[2] = speed_to_direction(stepper_x.speed());
}

/* Prompt for next X location until given int over Serial, then set x target location to the given position.
    Then prompt for next Y location; set y target locations to int given over Serial.
    @param xgo whether to accept the next x input (e.g. if x has reached its previous target destination)
    @param ygo whether to accept the next y input (e.g. if y has reached its previous target destination)
*/
void setNextPositionManually(bool xgo, bool ygo) {    // might be able to do a cleaner readUntil or something to prompt
    if ((receivedYPosition || !ygo) && receivedXPosition && xgo) {
        Serial.println("Next X Location:");
        receivedXPosition = false;
    }
    if (Serial.available()) {}
        int xPos = Serial.parseInt(); //read int or parseFloat for ..float...
        receivedXPosition = true;
        setXPos(xPos);
    }
    if ((receivedXPosition && !xgo) && receivedYPosition && ygo) {
        Serial.println("Next Y Location:");
        receivedYPosition = false;
    }
    if (Serial.available()) {}
        int yPos = Serial.parseInt(); //read int or parseFloat for ..float...
        receivedYPosition = true;
        setYPos(yPos);
    }
}

/* Set the target x position to X_MIN_POS <= position <= X_MAX_POS, or the closest bound
    @param the desired x position
*/
void setXPos(int position) {
    stepper_x.moveTo(min(position, X_MAX_POS));
    stepper_x.moveTo(max(position, X_MIN_POS);
}

/* Set the target y position to Y_MIN_POS <= position <= Y_MAX_POS, or the closest bound
    @param the desired y position
*/
void setYPos(int position) {
    multistepper_y.moveTo(min(position, Y_MAX_POS));
    multistepper_y.moveTo(max(position, Y_MIN_POS);
}

void setNextPositionManuallyPrompted() {
    setNextPositionManually(stepper_x.distanceToGo() == 0, multistepper_y.distanceToGo() == 0);
}

void setSpeedManually() {
    Serial.println("Speed (steps/min to run steppers at:");
    while (Serial.available() == 0);
    int desiredSpeed = Serial.parseInt(); //read int or parseFloat for ..float...
    if (desiredSpeed > MAX_SPEED){
        stepper_x.setMaxSpeed(MAX_SPEED);
        Serial.println("Too high! Speed set to default maximum speed.");
    } else if (desiredSpeed < MIN_SPEED) {
        stepper_x.setMaxSpeed(MIN_SPEED);
        Serial.println("Too low! Speed set to default minimum speed.");
    } else {
        stepper_x.setMaxSpeed(desiredSpeed);
    }
}

void sendStatus() {
	if (millis() - prev_millis > updateIntervalMilliseconds) {
	    switchLight();
		prev_millis = millis();
		set_status_packet();
		send_packet(&packet);
		Serial.print("\nLimit switches: ");
        Serial.print(packet->limit);    // can we see whether limit switches are triggered?
	}
}

/**
 * Main loop. Runs repeatedly while the board is on.
 *
*/
void loop() {
    sendStatus();

    setNextPositionManually(true, true);
    // or setNextPositionManuallyPrompted();
    // TODO: receive and execute input from pi/robotController

	stepper_x.run();
	multistepper_y.run();
}


