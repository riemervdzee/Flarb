// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_controller/controllers/cAvoidObstacle.h"

// Functions executed at the beginning and end of the Application
bool cAvoidObstacle::Create()
{
	return true;
}

void cAvoidObstacle::Destroy()
{

}

// Passes reference of "msg", is used as output
// Executes the AvoidObstacle sub-controller based on the rest of the arguments
// TODO maybe more parameters?
bool cAvoidObstacle::Execute( flarb_controller::WaypointVector &msg, const cImage &image)
{
	// Check if we are free
	int ret = image.CountBlockedRectangle( 186, 186, 140, 140);

	// There are no pixels in the area
	if( ret == 0)
	{
		return false;
	}
	else
	{
		// Set wanted-velocity to 0
		msg.x = msg.y = 0;
		return true;
	}
}

