// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_compass/cController.h"
#include "flarb_compass/Compass.h"

// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	_Compass = _rosNode.advertise<flarb_compass::Compass>("sensor/compass", 100);
	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	//now shutdown ros
	ros::shutdown();
}

// Updates the controller obj
void cController::Update()
{
	flarb_compass::Compass msg;
	msg.north_angle = 100;
	_Compass.publish(msg);
}
