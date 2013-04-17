// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_controller/controllers/cFollowSegment.h"

// Functions executed at the beginning and end of the Application
bool cFollowSegment::Create()
{
	return true;
}

void cFollowSegment::Destroy()
{

}

// Passes reference of "msg", is used as output
// Executes the FollowSegment sub-controller based on the rest of the arguments
// TODO maybe more parameters?
bool cFollowSegment::Execute( flarb_controller::WaypointVector &msg, const cImage &image)
{
	// TODO Do magic trick to get a way-point
	msg.x =   0;
	msg.y = -20;

	return true;
}
