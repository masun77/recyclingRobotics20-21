/*
 * Tests to make sure all of the motors can move in both of their respective directions simultaneously.
 * The cart will travel back and forth between the 0 and 400 steps positions.
 * Instructions: position the cart in the approximate middle of the x-axis
 * Future: Make the robot draw a sort of elliptical shape.
 */

#include <inttypes.h>
#include <MultiStepper.h>    // Allows control of multiple steppers at once
#include <AccelStepper.h>

#include "StepperController.h"
#include "pins.h"
#include "Communication.h"

state_t state;
packet_send_t packet;
unsigned long prev_millis = 0;
int updateIntervalMilliseconds = 250;
float maximumSpeed = 2000;
float minimumSpeed = 25;
bool limitSwitchTriggered;
int count = 1;

MultiStepper multistepper_y;
AccelStepper stepper_y1(AccelStepper::DRIVER, STR3_Y1_STEP, STR3_Y1_DIR, 0, 0, false);
AccelStepper stepper_y2(AccelStepper::DRIVER, STR3_Y2_STEP, STR3_Y2_DIR, 0, 0, false);
AccelStepper stepper_x(AccelStepper::DRIVER, STR3_X_STEP, STR3_X_DIR, 0, 0, false);

// TODO comments
void setStepperEnablePins() {
  stepper_y1.setEnablePin(STR3_Y1_EN);
  stepper_y2.setEnablePin(STR3_Y2_EN);
  stepper_x.setEnablePin(STR3_X_EN);
}

void invertPins() {
  stepper_y1.setPinsInverted(true, true, true);
  stepper_y2.setPinsInverted(true, true, true);
  stepper_x.setPinsInverted(true, true, true);
}

void createMultiStepperY() {
  multistepper_y.addStepper(stepper_y1);
  multistepper_y.addStepper(stepper_y2);
}

void setXstepper() {
  stepper_x.setMaxSpeed(maximumSpeed);
  stepper_x.setAcceleration(1200);
  stepper_x.enableOutputs();
}

void setYsteppers(){
  stepper_y1.setMaxSpeed(maximumSpeed);
  stepper_y1.setAcceleration(1000);
  stepper_y1.enableOutputs();
  stepper_y2.setMaxSpeed(maximumSpeed);
  stepper_y2.setAcceleration(1000);
  stepper_y2.enableOutputs();
}

void setLimitSwitches() {
  pinMode(LIM_X_MIN_A, INPUT_PULLUP);
  pinMode(LIM_X_MIN_B, INPUT_PULLUP);
  pinMode(LIM_X_MAX_A, INPUT_PULLUP);
  pinMode(LIM_X_MAX_B, INPUT_PULLUP);

  pinMode(LIM_Y_MIN_A, INPUT_PULLUP);
  pinMode(LIM_Y_MIN_B, INPUT_PULLUP);
  pinMode(LIM_Y_MAX_A, INPUT_PULLUP);
  pinMode(LIM_Y_MAX_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LIM_Y_MIN_A), limYMinAInterrupt, LOW);
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

void limYMinAInterrupt() {
  packet.limit = packet.limit | B10000;
  stepper_x.stop();
  limitSwitchTriggered = true;
  Serial.println("Switch triggered!");
  Serial.println(stepper_x.currentPosition());
}

void set_status_packet(packet_send_t* packet) {
  packet->control_state = state.control_state;  // todo: how can state get changed?
  packet->enabled = state.enabled;
  packet->limit = 0x0;    // todo: update based on limit_switches?
  packet->stepper_positions[0] = stepper_y1.currentPosition();
  packet->stepper_positions[1] = stepper_y2.currentPosition();
  packet->stepper_positions[2] = stepper_x.currentPosition();
  packet->stepper_directions[0] = speed_to_direction(stepper_y1.speed());
  packet->stepper_directions[1] = speed_to_direction(stepper_y2.speed());
  packet->stepper_directions[2] = speed_to_direction(stepper_x.speed());
}

void switchLight() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
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

boolean reachedYGoal() {
  if (stepper_y1.distanceToGo() == 0 && stepper_y2.distanceToGo() == 0){ return true; }
  else { return false;}
}

/**
   Main loop. Runs repeatedly while the board is on.
   UPDATE: The current code is for testing only, working on moving this to a separate file.
   The code uses blocking functions, so the packet will only be received after the stepper
   has moved one cycle (back and forth). The LED will not blink while program is running.
   TODO: receive and execute input
*/

/*
  When you power on the board or press reset, this function runs once.
*/
void setup() {
  setStepperEnablePins();
  invertPins();
  createMultiStepperY();
  setXstepper();
  setYsteppers();
  setLimitSwitches();
  blinkLight();
  setState();
  Serial.begin(115200);
  set_status_packet(&packet);
  blinkLight2();

  stepper_x.setCurrentPosition(0);
  stepper_y1.setCurrentPosition(0);
  stepper_y2.setCurrentPosition(0);
  stepper_x.moveTo(400);
  moveToY(200);
  stepper_x.setSpeed(400);
  setYSpeed(400);
  Serial.println(stepper_y1.distanceToGo());
  Serial.println(stepper_y2.distanceToGo());
  
}
  

void loop() {

  if (millis() - prev_millis > updateIntervalMilliseconds) {
    switchLight();
    prev_millis = millis();
//    set_status_packet(&packet);
//    send_packet(&packet);
  }

  stepper_x.run();
  runY();
  Serial.println(stepper_y1.currentPosition());
  if (reachedYGoal() && stepper_x.distanceToGo() == 0) {
    Serial.println("Curr Pos:");
    Serial.println(stepper_y1.currentPosition());
    stepper_x.moveTo(400 * (count%2));
    moveToY(200 * (count%2));
    Serial.println("Distance to Go:");
    Serial.println(stepper_y1.distanceToGo());
    count++;
    delay(250);
  }
  
}
