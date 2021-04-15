/* This is meant to check that the limit switches are connected and working properly 
 *  Author: Isabelle
 *  Last Modified: 12/14/20
 *  
 */

#include "pins.h"
 
// initialize switch states as off
int X1_min_state = 0;
int X1_max_state = 0;  
int X2_min_state = 0;
int X2_max_state = 0;   
int Y1_min_state = 0;
int Y2_min_state = 0;
int Y1_max_state = 0;
int Y2_max_state = 0;    

void setLimitSwitches() {
  pinMode(LIM_X1_MIN, INPUT_PULLUP);
  pinMode(LIM_X1_MAX, INPUT_PULLUP);
  pinMode(LIM_X2_MIN, INPUT_PULLUP);
  pinMode(LIM_X2_MAX, INPUT_PULLUP);
  pinMode(LIM_Y1_MIN, INPUT_PULLUP);
  pinMode(LIM_Y2_MIN, INPUT_PULLUP);
  pinMode(LIM_Y1_MAX, INPUT_PULLUP);
  pinMode(LIM_Y2_MAX, INPUT_PULLUP);
}

void setup() {
  // initialize the switch pins as an input:
  setLimitSwitches();
  Serial.begin(9600);
}

void loop() {
  // read the states of each switch
  X1_min_state = digitalRead(LIM_X1_MIN);
  X1_max_state = digitalRead(LIM_X1_MAX);
  X2_min_state = digitalRead(LIM_X2_MIN);
  X2_max_state = digitalRead(LIM_X2_MAX);
  Y1_min_state = digitalRead(LIM_Y1_MIN);
  Y2_min_state = digitalRead(LIM_Y2_MIN);
  Y1_max_state = digitalRead(LIM_Y1_MAX);
  Y2_max_state = digitalRead(LIM_Y2_MAX);

  String on_list = String();

  String text_disp = "";

  if(X1_min_state == 0){
    on_list.concat("  X1_min");
  }
  if(X1_max_state == 0){
    on_list.concat("  X2_max");
  }
  if(X2_min_state == 0){
    on_list.concat("  X2_min");
  }
  if(X2_max_state == 0){
    on_list.concat("  X2_max");
  }
  if(Y1_min_state == 0){
    on_list.concat("  Y1_min");
  }
  if(Y2_min_state == 0){
    on_list.concat("  Y2_min");
  }
  if(Y1_max_state == 0){
    on_list.concat("  Y1_max");
  }
  if(Y2_max_state == 0){
    on_list.concat("  Y2_max");
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
