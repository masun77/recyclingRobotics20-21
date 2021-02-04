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
volatile byte limitSwitchTriggered = B00000000;
bool minLimitSwitchTriggered = false;
bool startPressed = false;
bool stopPressed = false;
bool homing = false;
bool x_homed = false;
bool y_homed = false;
int next_state = 1;
int robot_state;
int goal;

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

  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(STOP_BUTTON, INPUT_PULLUP);
}

void setStartStopInterrupts() {
  attachInterrupt(digitalPinToInterrupt(START_BUTTON), startInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(STOP_BUTTON), stopInterrupt, FALLING);
}

void startInterrupt(){
  startPressed = true;
  Serial.println("START");
}

void stopInterrupt (){
  stopPressed = true;
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
  minLimitSwitchTriggered = true;
}

// Mark that this limit switch was triggered
void limXMinBInterrupt() {
  limitSwitchTriggered = limitSwitchTriggered | B10;
  minLimitSwitchTriggered = true;
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
  minLimitSwitchTriggered = true;
  
}

// Mark that this limit switch was triggered
void limYMinBInterrupt() {
  limitSwitchTriggered = limitSwitchTriggered | B100000;
  minLimitSwitchTriggered = true;
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

void setSpeedManually() {
  Serial.println("Speed (steps/min to run steppers at:");
  while (Serial.available() == 0);
  int desiredSpeed = Serial.parseInt(); //read int or parseFloat for ..float...
  if (abs(desiredSpeed) > MAX_SPEED) {
    stepper_x.setSpeed(MAX_SPEED);
    Serial.println("Too high! Speed set to default maximum speed.");
  } else if (abs(desiredSpeed) < MIN_SPEED) {
    stepper_x.setSpeed(MIN_SPEED);
    Serial.println("Too low! Speed set to default minimum speed.");
  } else {
    stepper_x.setSpeed(desiredSpeed);
  }
  Serial.print(stepper_x.speed());
}

void setYSpeed(int desiredSpeed){
  stepper_y1.setSpeed(desiredSpeed);
  stepper_y2.setSpeed(desiredSpeed);
}

void moveToY(long absolute){
  stepper_y1.moveTo(absolute);
  stepper_y2.moveTo(absolute);
}

void moveY(long relative){
  stepper_y1.move(relative);
  stepper_y2.move(relative);
}

void runY(){
  stepper_y1.run();
  stepper_y2.run();
}

void runSpeedY(){
  stepper_y1.runSpeed();
  stepper_y2.runSpeed();
}

boolean reachedYGoal() {
  if (stepper_y1.distanceToGo() == 0 && stepper_y2.distanceToGo() == 0){ return true; }
  else { return false;}
}

void setYsteppers(){
  stepper_y1.setMaxSpeed(MAX_SPEED);
  stepper_y1.setAcceleration(1000);
  stepper_y1.enableOutputs();
  stepper_y2.setMaxSpeed(MAX_SPEED);
  stepper_y2.setAcceleration(1000);
  stepper_y2.enableOutputs();
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
    setStartStopInterrupts();
	Serial.begin(115200);

    setState(CONTROL_STOPPED, false);
    set_status_packet();
    blinkLight();    // Hi there

  robot_state = WAITING_TO_START;
//	home();   // todo change to running on start button interrupt
//	setSpeedManually();    // todo: remove
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
//		set_status_packet();
//		send_packet(&packet);    // todo: do we get this packet?
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

  if(stopPressed || (limitSwitchTriggered && !homing)){
    robot_state = STOP;
  }
  Serial.print("ROBOT STATE: ");
  Serial.println(robot_state);
  switch (robot_state) {
    case WAITING_TO_START:
      homing = true;
      if (startPressed){
        startPressed = false;
        if ((!digitalRead(LIM_X_MIN_A) or !digitalRead(LIM_X_MIN_B)) && !x_homed){
          Serial.println("NEED TO MOVE X");
          stepper_x.move(15);
          stepper_x.run();
          x_homed = true; 
          next_state = HOME;
          robot_state = MOVING;
        } else if (!digitalRead(LIM_X_MAX_A) or !digitalRead(LIM_X_MAX_B)){
          stepper_x.move(-15);
          stepper_x.run();
          robot_state = MOVING;
        }
        if ((!digitalRead(LIM_Y_MIN_A) or digitalRead(!LIM_Y_MIN_B)) && !y_homed){
          moveY(15);
          runY();
          y_homed = true;
        } else if (!digitalRead(LIM_Y_MAX_A) or !digitalRead(LIM_Y_MAX_B)){
          moveY(-15);
          runY();
          robot_state = MOVING;
        }
        robot_state = HOME;
      }
      break;
    case HOME:
      if(minLimitSwitchTriggered){
        Serial.println("MIN SWITCH HIT");
        minLimitSwitchTriggered = false;
        robot_state = LS_HIT_HOMING;
        next_state = HOME;
      }
      
      // Setting XMin Position
      if (!x_homed){
        stepper_x.setSpeed(-MIN_SPEED);// todo: which limit switch are we going to?
        stepper_x.runSpeed();
      } 
      
     // Setting YMin Positions (FIGURE THIS OUT LATER) 
     Serial.println(!y_homed);
     if (!y_homed){
       Serial.println("Need to Home Y");
       setYSpeed(-MIN_SPEED);
       runSpeedY();
     // todo: stop the motor that gets there first. move 5 steps back, set position
     }
      next_state = SET_HOME;   
      break;
    case LS_HIT_HOMING:
      if (!digitalRead(LIM_X_MIN_A) or !digitalRead(LIM_X_MIN_B) && !x_homed){
        Serial.println("NEED TO MOVE X");
        stepper_x.stop();
        stepper_x.move(15);
        stepper_x.run();
        x_homed = true; 
        robot_state = MOVING;
        break;
      }
      if (!digitalRead(LIM_Y_MIN_A) or !digitalRead(LIM_Y_MIN_B) && !y_homed){
        stepper_y1.stop();
        stepper_y2.stop();
        moveY(15);
        runY();
        y_homed = true;
        robot_state = MOVING;
        break;
      }
      if (y_homed && x_homed){
        next_state = SET_HOME;
        robot_state = MOVING;
        break;
      }
      // todo: move motor 5 steps back, set position
      Serial.print("LIMIT SWITCH: ");
      Serial.println(limitSwitchTriggered);
      minLimitSwitchTriggered = false;
      limitSwitchTriggered = B00000000;
      robot_state = HOME;
      break;
    case MOVING:
      // could also use distanceToGo()
      Serial.print("DISTANCE TO GO: ");
      Serial.println(stepper_x.distanceToGo());
      stepper_x.run();
      runY();
      if (homing){
        if (minLimitSwitchTriggered){
          Serial.println("Going to LS_HIT_HOMING");
          robot_state = LS_HIT_HOMING; 
        } else {
          stepper_x.run();
          runY();
        }
      }
     if (stepper_x.distanceToGo() == 0 && stepper_y1.distanceToGo() == 0){
          Serial.print("Going to ");
          Serial.println(robot_state);
          robot_state = next_state;
     }
      break;
    case STOP:
      stepper_x.stop();
      stepper_y1.stop();
      stepper_y2.stop();
      robot_state = WAITING_TO_START;
      break;
    case SET_HOME:
      stepper_x.setCurrentPosition(0);
      stepper_y1.setCurrentPosition(0);
      stepper_y2.setCurrentPosition(0);
      stepper_x.stop();
      stepper_y1.stop();
      stepper_y2.stop();
      x_homed = true;
      y_homed = true;
      robot_state = WAITING_FOR_INSTRUCTION;
      break;
    case WAITING_FOR_INSTRUCTION:
      // Just wait for now
      limitSwitchTriggered = B00000000;
      Serial.write(limitSwitchTriggered);
      homing = false;

      Serial.println("Position for X Direction?")
      while(Serial.available() == 0){
      }
      xDir = Serial.parseInt();
      Serial.println("The X stepper will move:")
      Serial.println(xDir);

      Serial.println("Position for Y Direction?")
      while(Serial.available() == 0){
      }
      yDir = Serial.parseInt();
      Serial.println("The Y stepper will move:")
      Serial.println(yDir);

      stepper_x.moveTo(xDir);
      stepper_x.run();
      multistepper_y.moveTo(yDir);
      multistepper_y.run();

      next_state = WAITING_FOR_INSTRUCTION;
      robot_state = MOVING;
  }

 
}
