// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_example/cController.h"

// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	// Topic name / buffer
	_rosTopic = _rosNode.advertise<std_msgs::String>( "images", 100);

	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{

}

// Updates the controller obj
void cController::Update()
{
	// Increase count
	_count++;

	// Assemble message
	std_msgs::String msg;
	std::stringstream ss;
	ss << "hello world " << _count;
	msg.data = ss.str();

	// Publish
	_rosTopic.publish( msg);
}
