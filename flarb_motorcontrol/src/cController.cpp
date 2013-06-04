// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "flarb_motorcontrol/cController.h"
using namespace std;

// Functions executed at the beginning and end of the Node
bool cController::Create( int hz)
{
	// TODO now we just wait 1 sec before braking, maybe use other number?
	_framesToWait = hz;

	// RosCom
	_roscom.Create( &_rosNode);
	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	_roscom.Destroy();
}

// Resets counter
void cController::WaypointReceived()
{
	_framesWaiting = 0;
}

// Updates the controller obj
void cController::Update()
{
	_framesWaiting++;
	if( _framesWaiting == _framesToWait)
	{
		_roscom.MotorBrake();
	}
}
