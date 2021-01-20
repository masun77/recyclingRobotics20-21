/* This is meant to check that the limit switches are connected and working properly 
 *  Author: Isabelle
 *  Last Modified: 12/14/20
 *  
 *  NOTE: NOT TESTED
 */

#include "pins.h"
 
// initialize switch states as off
int X_minA_state = 0;
int X_maxA_state = 0;
int X_minB_state = 0;
int X_maxB_state = 0;
int Y_minA_state = 0;
int Y_maxA_state = 0;
int Y_minB_state = 0;
int Y_maxB_state = 0;         

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

void setup() {
  // initialize the switch pins as an input:
  setLimitSwitches();
  Serial.begin(9600);
}

void loop() {
  // read the states of each switch
  X_minA_state = digitalRead(LIM_X_MIN_A);
  X_maxA_state = digitalRead(LIM_X_MAX_A);
  X_minB_state = digitalRead(LIM_X_MIN_B);
  X_maxB_state = digitalRead(LIM_X_MAX_B);
  Y_minA_state = digitalRead(LIM_Y_MIN_A);
  Y_maxA_state = digitalRead(LIM_Y_MAX_A);
  Y_minB_state = digitalRead(LIM_Y_MIN_B);
  Y_maxB_state = digitalRead(LIM_Y_MAX_B);

  String on_list = String();

  String text_disp = "";

  if(X_minA_state == 0){
    on_list.concat("  X_min");
  }
  if(X_maxA_state == 0){
    on_list.concat("  X_max");
  }
  if(X_minB_state == 0){
    on_list.concat("  X_min");
  }
  if(X_maxB_state == 0){
    on_list.concat("  X_max");
  }
  if(Y_minA_state == 0){
    on_list.concat("  Y1_min");
  }
  if(Y_maxA_state == 0){
    on_list.concat("  Y1_max");
  }
  if(Y_minB_state == 0){
    on_list.concat("  Y1_min");
  }
  if(Y_maxB_state == 0){
    on_list.concat("  Y1_max");
  }

  // Display which buttons are pressed
  if (on_list.length() < 1){
    text_disp = "None of the switches are pressed.";
  }
  else{
    text_disp = String("The following switch[es] are pressed:" + on_list);
  }

  Serial.println(text_disp);

  delay(250);

}
