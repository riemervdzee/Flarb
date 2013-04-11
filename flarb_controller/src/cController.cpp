// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_controller/cController.h"
#include "flarb_controller/cMovement.h"

// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	cMovement Move;
	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{

}

// Updates the controller obj
void cController::Update()
{
	
}
