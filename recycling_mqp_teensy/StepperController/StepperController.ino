#include <inttypes.h>    // todo add to instructions: this library

#include "MultiStepper.h"    // Allows control of multiple steppers at once (contains 4 functions)
#include "AccelStepper.h"
#include "StepperController.h"
#include "pins.h"
#include "Communication.h"

state_t state;
volatile packet_send_t packet;
unsigned long previous_milliseconds = 0;
bool receivedXPosition = true;
bool receivedYPosition = true;
volatile byte limitSwitchTriggered = 0;

MultiStepper multistepper_y;
AccelStepper stepper_y1(AccelStepper::DRIVER, STR3_Y1_STEP, STR3_Y1_DIR, 0, 0, false);
AccelStepper stepper_y2(AccelStepper::DRIVER, STR3_Y2_STEP, STR3_Y2_DIR, 0, 0, false);
AccelStepper stepper_x(AccelStepper::DRIVER, STR3_X_STEP, STR3_X_DIR, 0, 0, false);

// Blink the LED
void blinkLight() {
  digitalWrite(LED_PIN, LOW);
  delay(2000);
  digitalWrite(LED_PIN, HIGH);
}

// Switch the light to off if currently on; on if currently off.
void switchLight() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

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

// Create the combined stepper of the two Y-axis motors by adding
// the two steppers to the multistepper object
void assignMotorsToMultiStepperY() {
  multistepper_y.addStepper(stepper_y1);
  multistepper_y.addStepper(stepper_y2);
}

// Enable all 3 steppers and set their acceleration and max speed
void enableSteppers() {
    stepper_x.setMaxSpeed(MAX_SPEED);
	stepper_y1.setMaxSpeed(MAX_SPEED);    // Multistepper library doesn't have these functions
	stepper_y2.setMaxSpeed(MAX_SPEED);    // so have to call on the individual stepper motors
    stepper_x.setAcceleration(ACCELERATION);
	stepper_y1.setAcceleration(ACCELERATION);
	stepper_y2.setAcceleration(ACCELERATION);
    stepper_x.enableOutputs();
	stepper_y1.enableOutputs();
	stepper_y2.enableOutputs();
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

// Attach appropriate interrupt function to each limit switch pin
void setLimitSwitchInterrupts() {
  attachInterrupt(digitalPinToInterrupt(LIM_X_MIN_A), limXMinAInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIM_X_MIN_B), limXMinBInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIM_X_MAX_A), limXMaxAInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIM_X_MAX_B), limXMaxBInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIM_Y_MIN_A), limYMinAInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIM_Y_MIN_B), limYMinBInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIM_Y_MAX_A), limYMaxAInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIM_Y_MAX_B), limYMaxBInterrupt, FALLING);
}

// todo: button interrupts

// Mark that this limit switch was triggered
void limXMinAInterrupt() {    // todo make motor and pin names correspond nicely
  limitSwitchTriggered = limitSwitchTriggered | B1;
}

// Mark that this limit switch was triggered
void limXMinBInterrupt() {
  limitSwitchTriggered = limitSwitchTriggered | B10;
}

// Mark that this limit switch was triggered
void limXMaxAInterrupt() {
  limitSwitchTriggered = limitSwitchTriggered | B100;
}

// Mark that this limit switch was triggered
void limXMaxBInterrupt() {
  limitSwitchTriggered = limitSwitchTriggered | B1000;
}

// Mark that this limit switch was triggered
void limYMinAInterrupt() {
  limitSwitchTriggered = limitSwitchTriggered | B10000;
}

// Mark that this limit switch was triggered
void limYMinBInterrupt() {
  limitSwitchTriggered = limitSwitchTriggered | B100000;
}

// Mark that this limit switch was triggered
void limYMaxAInterrupt() {
  limitSwitchTriggered = limitSwitchTriggered | B1000000;
}

// Mark that this limit switch was triggered
void limYMaxBInterrupt() {
  limitSwitchTriggered = limitSwitchTriggered | B10000000;
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

/* Set the status packet to send to the main robot controller.
*/
void set_status_packet() {
  packet.control_state = state.control_state;
  packet.enabled = state.enabled;
  packet.stepper_positions[0] = stepper_y1.currentPosition();
  packet.stepper_positions[1] = stepper_y2.currentPosition();
  packet.stepper_positions[2] = stepper_x.currentPosition();
  packet.stepper_directions[0] = speed_to_direction(stepper_y1.speed());
  packet.stepper_directions[1] = speed_to_direction(stepper_y2.speed());
  packet.stepper_directions[2] = speed_to_direction(stepper_x.speed());
  packet.limit = limitSwitchTriggered;
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

// Send the x and y steppers to corresponding limit switches to establish their positions
void home() {
    stepper_x.setSpeed(MIN_SPEED);
    while (!(limitSwitchTriggered & B1111)) {    // todo: which limit switch are we going to?
        stepper_x.runSpeed();
    }
    // todo: move motor one step back, set position
    limitSwitchTriggered = 0;

    stepper_y1.setSpeed(MIN_SPEED);
    stepper_y2.setSpeed(MIN_SPEED);
    while (!(limitSwitchTriggered & B11110000)) {    // todo: which lim switch run to and how deal w 2 motors?
        stepper_y1.runSpeed();
        stepper_y2.runSpeed();
        // todo: stop the motor that gets there first. move one step back, set position
    }
    limitSwitchTriggered = 0;
}

void setSpeedManually() {
  Serial.println("Speed (steps/min to run steppers at:");
  while (Serial.available() == 0);
  int desiredSpeed = Serial.parseInt(); //read int or parseFloat for ..float...
  if (desiredSpeed > MAX_SPEED) {
    stepper_x.setSpeed(MAX_SPEED);
    Serial.println("Too high! Speed set to default maximum speed.");
  } else if (desiredSpeed < MIN_SPEED) {
    stepper_x.setSpeed(MIN_SPEED);
    Serial.println("Too low! Speed set to default minimum speed.");
  } else {
    stepper_x.setSpeed(desiredSpeed);
  }
}

/*
When you power on the board or press reset, this function runs once.
Sets up the pins, steppers, limit switches, and state. Begins Serial connection.
*/
void setup() {
	pinMode(LED_PIN, OUTPUT);    // Set the LED pin to be ready to blink
    blinkLight();    // Just so you know it's on
    setStepperEnablePins();
    invertPins();
    assignMotorsToMultiStepperY();
    enableSteppers();
    setLimitSwitches();
    setLimitSwitchInterrupts();
	Serial.begin(115200);

    setState(CONTROL_STOPPED, false);
    set_status_packet();
    blinkLight();    // Hi there

	home();   // todo change to running on start button interrupt
	setSpeedManually();    // todo: remove
}


/* Set the target x position to the input position or the closest bound
    @param the desired x position
*/
void set_x_position(int position) {
  long int pos = min(position, X_MAX_POS);
  pos = max(position, X_MIN_POS);
  stepper_x.moveTo(pos);
}

/* Set the target y position to the input position or the closest bound
    @param the desired y position
*/
void set_y_position(int position) {
  long int pos = min(position, Y_MAX_POS);
  pos = max(position, Y_MIN_POS);
  long int positions[] = {pos, pos};
  multistepper_y.moveTo(positions);
}

// todo: fix or remove
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
    if (Serial.available()) {
        int xPos = Serial.parseInt(); //read int or parseFloat for ..float...
        receivedXPosition = true;
        set_x_position(xPos);
    }
    if ((receivedXPosition && !xgo) && receivedYPosition && ygo) {
        Serial.println("Next Y Location:");
        receivedYPosition = false;
    }
    if (Serial.available()) {
        int yPos = Serial.parseInt(); //read int or parseFloat for ..float...
        receivedYPosition = true;
        set_y_position(yPos);
    }
}

void setNextPositionManuallyPrompted() {
    setNextPositionManually(stepper_x.distanceToGo() == 0, true); // todo fix second arg
}

// At regular time intervals, send a status packet over the serial connection
void sendStatus() {
    unsigned long ms = millis() - previous_milliseconds;
	if (ms > UPDATE_INTERVAL_MILLISECONDS) {
	    switchLight();
		previous_milliseconds = millis();
		set_status_packet();
		send_packet(&packet);    // todo: do we get this packet?
		Serial.print("\nLimit switches: ");
        Serial.print(packet.limit);
	}
}

/**
 * Main loop. Runs repeatedly while the board is on.
 *
*/
void loop() {
    // todo: use the state helpfully - e.g. if state is not enabled, disable motors

    sendStatus();

    if (limitSwitchTriggered > 0) {
        // todo: stop the appropriate motor(s)
        // move it away from the limit switch
        // reset its current position
        // what should happen next? stop entirely or continue following instructions?
        limitSwitchTriggered = 0;
    }

    // for now what we're doing here is manually inputting positions
    // TODO: receive and execute input from pi/robotController

	stepper_x.run();
	multistepper_y.run();
}
