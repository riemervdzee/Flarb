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
bool cFollowSegment::Execute( tVector &vector, cMap &map)
{
	// First check if there is room at both sides
	tBoundingBox b( tVector( 0, -0.5f), tVector( 0.8f, 0.5f));
	if(map.CheckIntersectionRegion( b))
	{
		cout << "FollowSegment exiting" << endl;
		return false;
	}

	// Just continue the current row
	tVector direction = tVector( 0.0f, 0.5f);
	tVector output;
	float result = map.FindFreePath( FLARB_EXTRA_RADIUS, direction, output);

	// TODO here we can do stuff with the result, whether 0 or negative 
	// return false maybe? For now, just set vector = output
	vector = output;

	return true;
}
