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
	// Check if we are free, note: offset_width is used at both sides
	int offset_width  = 0.20f * (image.getMapImage()->imageX/image.getMapImage()->sizeWidth);
	int height        = 0.40f * (image.getMapImage()->imageY/image.getMapImage()->sizeHeight);
	int width = offset_width * 2;

	int x0 = image.getMapImage()->cameraX - offset_width;
	int y0 = image.getMapImage()->cameraY - height;

	int ret = image.CountBlockedRectangle( x0, y0, width, height);

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

