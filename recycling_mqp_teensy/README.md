# recycling_mqp_teensy

The Teensy code files consist of a class to control and interface
with the stepper motors (StepperController.ino and .h), a class to control communications
with the Raspberry Pi (Communications.cpp and .h), and a pins.h file
defining what the pins on the Teensy correspond to.

The StepperController.ino file is the main file that runs. The StepperController controls the stepper motors
which move the robot platform in the x plane (across the conveyor belt) and the
y plane (parallel to the conveyor belt's movement). Its setup() function runs first (when the board is powered on
or reset), followed repeatedly by the loop() function (runs over and over until the board is turned off).
In the loop, StepperController sends an update to the Pi using the Communication class. It also checks for instructions from
the Pi... todo


TODO: buttonController class