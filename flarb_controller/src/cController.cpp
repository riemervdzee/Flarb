// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_controller/cController.h"

// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	// Init RosCom object
	_roscom.Create( &_rosNode);
	
	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	_roscom.Destroy();
}

// Updates the controller obj
void cController::Update()
{
	
}
