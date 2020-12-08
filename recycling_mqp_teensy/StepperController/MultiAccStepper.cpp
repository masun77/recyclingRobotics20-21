// MultiStepper.cpp
//
// Copyright (C) 2015 Mike McCauley
// $Id: MultiStepper.cpp,v 1.3 2020/04/20 00:15:03 mikem Exp mikem $

#include "MultiAccStepper.h"
#include "AccelStepper.h"

MultiAccStepper::MultiAccStepper()
    : _num_steppers(0)
{
}

boolean MultiAccStepper::addStepper(AccelStepper& stepper)
{
    if (_num_steppers >= MULTIACCSTEPPER_MAX_STEPPERS)
	return false; // No room for more
    _steppers[_num_steppers++] = &stepper;
    return true;
}

/*
* Move to an absolute position where negative is CCW from 0 position
* @param the desired position 
*/
void MultiAccStepper::moveTo(long absolute)
{
	uint8_t i;
	for (i = 0; i < _num_steppers; i++)
	{
	    _steppers[i]->moveTo(absolute[i]); // New target position 
	}
    }
}


/*
* Move to a position relative to current position
* @param the desired relative
*/
void MultiAccStepper::move(long relative)
{
	uint8_t i;
	for (i = 0; i < _num_steppers; i++)
	{
	    _steppers[i]->moveTo(relative); // New target position 
	}
}


/*
* Polls motor to step if need to reach the target position.
* @return true if the any motor is still running
*/
boolean MultiAccStepper::run()
{
    uint8_t i;
    boolean ret = false;
    for (i = 0; i < _num_steppers; i++)
    {
	if ( _steppers[i]->distanceToGo() != 0)
	{
	    _steppers[i]->run();
	    ret = true;
	}
	// Caution: it has een reported that if any motor is used with acceleration outside of
	// MultiStepper, this code is necessary, you get 
	// strange results where it moves in the wrong direction for a while then 
	// slams back the correct way.
#if 0
	else
	{
	    // Need to call this to clear _stepInterval, _speed and _n 
	    otherwise future calls will fail.
		_steppers[i]->setCurrentPosition(_steppers[i]->currentPosition());
	}
#endif
	
    }
    return ret;
}

/*
* Move at a constant specified speed
* @param the desired constant speed
*/
void MultiAccStepper::runSpeed(float speed)
{
	setSpeed(speed)
	uint8_t i;
	for (i = 0; i < _num_steppers; i++)
	{
	    _steppers[i]->runSpeed(); 
	}
	run();
}

// Blocks until all steppers reach their target position and are stopped
void MultiAccStepper::runSpeedToPosition()
{ 
    while (run())
	;
}

/*
* Sets the acceleration of both motors
* @param the desired acceleration 
*/
void setAcceleration(float acceleration)
{
	uint8_t i;
	for (i = 0; i < _num_steppers; i++)
	{
	    _steppers[i]->setAcceleration(acceleration); // New acceleration
	}
}

/*
* Specifies the speed
* @param the desired speed 
*/
void setSpeed(float speed)
{
	uint8_t i;
	for (i = 0; i < _num_steppers; i++)
	{
	    _steppers[i]->setSpeed(speed); // New speed
	}
}

/*
* Stops all of the motors
*/
void stop()
{
	uint8_t i;
	for (i = 0; i < _num_steppers; i++)
	{
	    _steppers[i]->stop(); 
	}
}


bool reachedGoal()
{
	bool runningStatus = false;
	uint8_t i;
	for (i = 0; i < _num_steppers; i++)
	{
	    long distance = _steppers[i]->distanceToGo(speed); // steps to go
		if (distance != 0){
			runningStatus = true;
		}
	}
	return runningStatus;
}
}
