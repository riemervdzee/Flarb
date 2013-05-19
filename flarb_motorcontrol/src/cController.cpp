// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <cmath>

#include "ros/ros.h"

#include "flarb_motorcontrol/cController.h"

// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	_roscom.Create( &_rosNode, this);
	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	_roscom.Destroy();
}

// Calculates motor strengths based on input
void cController::SetWaypoint( float x, float y)
{
	// TODO scale motor strengths to wanted size
	float AlphaRadians = atan2( y, x);
	float SpeedFactor  = sqrt(x*x + y*y*4) / 2;
	float R       = AlphaRadians / M_PI;
	float L	      = (AlphaRadians < 0) ? (-1 - R) : (1-R);
	_inputLeft    = R * SpeedFactor;
	_inputRight   = L * SpeedFactor;
}

// Updates the controller obj
void cController::Update()
{
	// TODO PID control..
	_roscom.SendMotorStrength( (int)_inputLeft, (int)_inputRight);
}
