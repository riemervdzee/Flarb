// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include "ros/ros.h"
#include "flarb_controller/controllers/cFollowSegment.h"
using namespace std;

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
	// Basic test of the area we are scanning in
	int offset_width  = 0.50f * image.getMeters2Pixels();
	int height        = 0.50f * image.getMeters2Pixels();
	int width = offset_width * 2;

	int x0 = image.getMapImage()->cameraX - offset_width;
	int y0 = image.getMapImage()->cameraY;

	int ret = image.CountBlockedRectangle( x0, y0, width, height);

	msg.x = 0;
	msg.y = 0.5f;

	//
	if( ret == 0)
		return true;

	int sumLeft  = 0;
	int sumRight = 0;

	for(int i = 0; i < height; i++)
	{
		sumLeft  += image.GetXRight ( image.getMapImage()->cameraX, y0 + i);
		sumRight += image.GetXLeft  ( image.getMapImage()->cameraX, y0 + i);
	}

	// Average
	sumLeft  /= height;
	sumRight /= height;

	// Get average of these two again
	// TODO this is probably funky
	msg.x = (((sumLeft + sumRight) /2) - image.getMapImage()->cameraX) * image.getPixels2Meters();

	float length = 0.5f / sqrt(msg.x*msg.x + msg.y*msg.y);

	msg.x *= length;
	msg.y *= length;

	cout << msg.x << " "<< msg.y << endl;

	return true;
}
