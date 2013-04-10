// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_accelerometer/cController.h"
#include "flarb_accelerometer/Accelerometer.h"

// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	//Create publisher
	_Accelerometer = _rosNode.advertise<flarb_accelerometer::Accelerometer>("sensor/accelerometer", 100);
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
	//send random data
	flarb_accelerometer::Accelerometer msg;
	msg.Speed = 25;
	_Accelerometer.publish(msg);
}
