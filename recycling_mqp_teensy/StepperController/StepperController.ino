#include <inttypes.h>
#include "MultiStepper.h"    // Allows control of multiple steppers at once - todo consider elimintating this or writing own
#include "AccelStepper.h"    // Import into Arduino IDE

#include "StepperController.h"
#include "pins.h"
#include "Communication.h"

state_t state;
packet_send_t packet;
unsigned long prev_millis = 0;
float MAX_SPEED = 1000;
float MIN_SPEED = 25;
bool receivedXPosition = true;
bool receivedYPosition = true;
bool limitSwitchTriggered = false;
int ACCELERATION = 500;

int X_MAX_POS = 1500; // todo
int X_MIN_POS = 0;
int Y_MAX_POS = 1500;
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
void enableSteppers() {
    stepper_x.setMaxSpeed(maximumSpeed);
    stepper_x.setAcceleration(ACCELERATION);
    stepper_x.enableOutputs();
	stepper_y1.setMaxSpeed(MAX_SPEED);
	stepper_y2.setMaxSpeed(MAX_SPEED);
	stepper_y1.setAcceleration(ACCELERATION);
	stepper_y2.setAcceleration(ACCELERATION);
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

// Attach interrupt function to each limit switch pin
void setLimitSwitchInterrupts() {
  attachInterrupt(digitalPinToInterrupt(LIM_X_MIN_A), limXMinAInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(LIM_X_MIN_B), limXMinBInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(LIM_X_MAX_A), limXMaxAInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(LIM_X_MAX_B), limXMaxBInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(LIM_Y_MIN_A), limYMinAInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(LIM_Y_MIN_B), limYMinBInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(LIM_Y_MAX_A), limYMaxAInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(LIM_Y_MAX_B), limYMaxBInterrupt, LOW);
}

// todo: button interrupts

void limXMinAInterrupt() {
  packet.limit = packet.limit | B1;
  stepper_x.stop();
  stepper_x.setCurrentPosition(X_MIN_POS);
  // stepper_x.runSpeed  todo - move motor away from limit switch
  limitSwitchTriggered = true;
}

void limXMinBInterrupt() {
  packet.limit = packet.limit | B10;
  stepper_x.stop();
  stepper_x.setCurrentPosition(X_MIN_POS);
  limitSwitchTriggered = true;
}

void limXMaxAInterrupt() {
  packet.limit = packet.limit | B100;
  stepper_x.stop();
  stepper_x.setCurrentPosition(X_MAX_POS);
  limitSwitchTriggered = true;
}

void limXMaxBInterrupt() {
  packet.limit = packet.limit | B1000;
  stepper_x.stop();
  stepper_x.setCurrentPosition(X_MAX_POS);
  limitSwitchTriggered = true;
}

void limYMinAInterrupt() {    // todo make names correspond nicely
  packet.limit = packet.limit | B10000;
  stepper_y1.stop();
  stepper_y2.stop();
  stepper_y1.setCurrentPosition(Y_MIN_POS);   // todo only stop one
  stepper_y2.setCurrentPosition(Y_MIN_POS);
  limitSwitchTriggered = true;
}

void limYMinBInterrupt() {
  packet.limit = packet.limit | B100000;
  stepper_y1.stop();
  stepper_y2.stop();
  stepper_y1.setCurrentPosition(Y_MIN_POS);
  stepper_y2.setCurrentPosition(Y_MIN_POS);
  limitSwitchTriggered = true;
}

void limYMaxAInterrupt() {
  packet.limit = packet.limit | B1000000;
  stepper_y1.stop();
  stepper_y2.stop();
  stepper_y1.setCurrentPosition(Y_MAX_POS);
  stepper_y2.setCurrentPosition(Y_MAX_POS);
  limitSwitchTriggered = true;
}

void limYMaxBInterrupt() {
  packet.limit = packet.limit | B10000000;
  stepper_y1.stop();
  stepper_y2.stop();
  stepper_x.setCurrentPosition(Y_MAX_POS);
  stepper_y1.setCurrentPosition(Y_MAX_POS);
  stepper_y2.setCurrentPosition(Y_MAX_POS);
  limitSwitchTriggered = true;
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

void setSpeedManually() {
  Serial.println("Speed (steps/min to run steppers at:");
  while (Serial.available() == 0);
  int desiredSpeed = Serial.parseInt(); //read int or parseFloat for ..float...
  if (desiredSpeed > MAX_SPEED) {
    stepper_x.setMaxSpeed(MAX_SPEED);
    Serial.println("Too high! Speed set to default maximum speed.");
  } else if (desiredSpeed < MIN_SPEED) {
    stepper_x.setMaxSpeed(MIN_SPEED);
    Serial.println("Too low! Speed set to default minimum speed.");
  } else {
    stepper_x.setMaxSpeed(desiredSpeed);
  }
}

// Send steppers to limit switches to set their position
void home() {
  stepper_x.setSpeed(MIN_SPEED);
  while (!limitSwitchTriggered) {
    stepper_x.runSpeed();
  }
  limitSwitchTriggered = false;

  stepper_y1.setSpeed(MIN_SPEED);
  stepper_y2.setSpeed(MIN_SPEED);
  while (!limitSwitchTriggered) {
    stepper_y1.runSpeed();
    stepper_y2.runSpeed();
  }
  limitSwitchTriggered = false;
}

/*
When you power on the board or press reset, this function runs once.
Sets up the pins, steppers, limit switches, and state. Begins Serial connection.
*/
void setup() {
    setStepperEnablePins();
    invertPins();
    createMultiStepperY();
    enableSteppers();
    setLimitSwitches();
	pinMode(LED_PIN, OUTPUT);
    blinkLight();
    setState(CONTROL_STOPPED, false);
	Serial.begin(115200);
    set_status_packet();
	packet.limit = 0x0;  // Initial limit switch values all 0
    blinkLight();

    // todo: clean this up so there's a builder function for setup/easy to switch between setup strategies
	home();   // todo change to button interrupt
	setSpeedManually();
}

/* Set the status packet to send to the main robot controller.
    @ packet the address of the packet to send
*/
void set_status_packet() {
  packet.control_state = state.control_state;  // todo: how can state get changed?
  packet.enabled = state.enabled;
  packet.stepper_positions[0] = stepper_y1.currentPosition();
  packet.stepper_positions[1] = stepper_y2.currentPosition();
  packet.stepper_positions[2] = stepper_x.currentPosition();
  packet.stepper_directions[0] = speed_to_direction(stepper_y1.speed());
  packet.stepper_directions[1] = speed_to_direction(stepper_y2.speed());
  packet.stepper_directions[2] = speed_to_direction(stepper_x.speed());
}

/* Set the target x position to X_MIN_POS <= position <= X_MAX_POS, or the closest bound
    @param the desired x position
*/
void setXPos(int position) {
  long int pos = min(position, X_MAX_POS);
  pos = max(position, X_MIN_POS);
  stepper_x.moveTo(pos);
}

/* Set the target y position to Y_MIN_POS <= position <= Y_MAX_POS, or the closest bound
    @param the desired y position
*/
void setYPos(int position) {
  long int pos = min(position, Y_MAX_POS);
  pos = max(position, Y_MIN_POS);
  long int positions[] = {pos, pos};
  multistepper_y.moveTo(positions);
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
    if (Serial.available()) {
        int xPos = Serial.parseInt(); //read int or parseFloat for ..float...
        receivedXPosition = true;
        setXPos(xPos);
    }
    if ((receivedXPosition && !xgo) && receivedYPosition && ygo) {
        Serial.println("Next Y Location:");
        receivedYPosition = false;
    }
    if (Serial.available()) {
        int yPos = Serial.parseInt(); //read int or parseFloat for ..float...
        receivedYPosition = true;
        setYPos(yPos);
    }
}

void setNextPositionManuallyPrompted() {
    setNextPositionManually(stepper_x.distanceToGo() == 0, true); // todo fix second arg
}

void sendStatus() {
    unsigned long ms = millis() - prev_millis;
	if (ms > UPDATE_INTERVAL_MILLISECONDS) {
	    switchLight();
		prev_millis = millis();
		set_status_packet();
		send_packet(&packet);
		Serial.print("\nLimit switches: ");
        Serial.print(packet.limit);    // can we see whether limit switches are triggered?
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
