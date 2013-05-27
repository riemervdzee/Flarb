// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "flarb_motorcontrol/cController.h"
using namespace std;

// Functions executed at the beginning and end of the Node
bool cController::Create( int hz)
{
	// RosCom
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
	// Convert the vector to two motor strengths
	// TODO write this a bit more proper. Support turning!
	float AlphaRadians = atan2( y, x);
	float SpeedFactor  = sqrt(x*x + y*y*4) * VEC2MOTOR;
	float R       = AlphaRadians / M_PI;
	float L	      = (AlphaRadians < 0) ? (-1 - R) : (1-R);
	int _inputRight   = R * SpeedFactor;
	int _inputLeft    = L * SpeedFactor;

	// Send motor strengths
	_roscom.SendMotorStrength( (int)_inputLeft, (int)_inputRight, false);

#if 0
	cout << "[SET] Right " << _inputRight << ", Left " << _inputLeft << endl;
#endif
}

// Updates the controller obj
void cController::Update()
{
	// TODO after X updates with no SetWaypoint, brake the mofo thing
}
