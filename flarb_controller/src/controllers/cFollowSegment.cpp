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

// Passes reference of "vector", is used as output
// Executes the FollowSegment sub-controller based on the rest of the arguments
// TODO maybe more parameters?
bool cFollowSegment::Execute( tVector &vector, const cImage &image)
{
	// Basic test of the area we are scanning in
	int offset_width  = 0.50f * image.getMeters2Pixels();
	int height        = 0.50f * image.getMeters2Pixels();
	int width = offset_width * 2;

	int x0 = image.getMapImage()->cameraX - offset_width;
	int y0 = image.getMapImage()->cameraY;

	int ret = image.CountBlockedRectangle( x0, y0, width, height);

	vector = tVector( 0, 0.5f);

	//
	if( ret == 0)
		return true;

	int leftMax  = 0;
	int rightMin = 9999;

	for(int i = 0; i < height; i++)
	{
		int temp;
		temp = image.GetXRight ( image.getMapImage()->cameraX, y0 + i);
		if( temp < rightMin)
			rightMin = temp;

		temp = image.GetXLeft  ( image.getMapImage()->cameraX, y0 + i);
		if( temp > leftMax)
			leftMax = temp;
	}

	// Get average, substract the cam pos and conver it to WorldSpace units
	float avg    = (leftMax + rightMin) /2;
	float offset = avg - image.getMapImage()->cameraX;
	vector.setX( offset * image.getPixels2Meters() );

	// Set the length to 0.5f
	vector.setLength( 0.5f);

	//cout << leftMax << " "<< rightMin << ", avg " << avg << " " << offset << endl;
	//cout << vector.getX() << " "<< vector.getY() << endl << endl;

	return true;
}
