// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "flarb_motorcontrol/cController.h"
using namespace std;

// Functions executed at the beginning and end of the Node
bool cController::Create( int hz)
{
	// Set PIDs
	_pidLeft  = cPID( hz, DIR_DIRECT, -MOTOR_MAX, MOTOR_MAX, PID_KP, PID_KI, PID_KD);
	_pidRight = cPID( hz, DIR_DIRECT, -MOTOR_MAX, MOTOR_MAX, PID_KP, PID_KI, PID_KD);

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
	// TODO scale motor strengths to wanted size
	float AlphaRadians = atan2( y, x);
	float SpeedFactor  = sqrt(x*x + y*y*4);
	float R       = AlphaRadians / M_PI;
	float L	      = (AlphaRadians < 0) ? (-1 - R) : (1-R);
	_inputRight   = R * SpeedFactor * 100;
	_inputLeft    = L * SpeedFactor * 100;

	cout << "[SET] Right " << _inputRight << ", Left " << _inputLeft << endl;
}

// Updates the controller obj
void cController::Update()
{
	// TODO PID control..
	//_roscom.SendMotorStrength( (int)_inputLeft, (int)_inputRight);
	_outputRight = _pidRight.Execute( _inputRight, _outputRight);
	//_outputLeft  =  _pidLeft.Execute(  _inputLeft,  _outputLeft);

	cout << "[UPD] Right " << _outputRight << endl;// ", Left " << _outputLeft  << endl;
}
