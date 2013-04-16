// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_controller/cController.h"

// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	// Init RosCom object
	_rosCom.Create( &_rosNode, this);
	
	// Init sub-controllers
	_followSegment.Create();
	_findSegment.Create();
	_avoidObstacle.Create();

	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	// Deinit RosCom and subcontrollers
	_rosCom.Destroy();
	_followSegment.Destroy();
	_findSegment.Destroy();
	_avoidObstacle.Destroy();
}

// Updates the controller obj
void cController::Update()
{
	// TODO remove?
}

// Gets called by cRosCom when we received a /map message
void cController::CallbackMap( const cImage &image)
{
	// Delegate which sub-controller should get executed
}
