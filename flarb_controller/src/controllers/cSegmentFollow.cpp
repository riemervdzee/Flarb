// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include "ros/ros.h"
#include "flarb_controller/controllers/cSegmentFollow.h"
using namespace std;

// Functions executed at the beginning and end of the Application
bool cSegmentFollow::Create()
{
	return true;
}

void cSegmentFollow::Destroy()
{

}

void cSegmentFollow::Reinit( const flarb_msgs::State &state)
{

}

// Passes reference of "vector", is used as output
// Executes the SegmentFollow sub-controller based on the rest of the arguments
enum SUBRETURN cSegmentFollow::Execute( tVector &output, const flarb_msgs::State &state, cMap &map)
{
	// First check if there is room at both sides
	tBoundingBox b( tVector( -0.5f, 0), tVector( 0.5f, 0.2f));
	if(!map.CheckIntersectionRegion( b))
	{
		return RET_NEXT;
	}

	// Just continue the current row
	tVector direction = tVector( 0.0f, 0.5f);
	float result = map.FindFreePath( FLARB_EXTRA_RADIUS, direction, output, false);

	// TODO here we can do stuff with the result, whether 0 or negative 
	// return false maybe? For now, just set vector = output

	return RET_SUCCESS;
}
