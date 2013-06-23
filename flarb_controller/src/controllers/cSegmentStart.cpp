// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include "ros/ros.h"
#include "flarb_controller/controllers/cSegmentStart.h"

// Functions executed at the beginning and end of the Application
bool cSegmentStart::Create()
{
	return true;
}

void cSegmentStart::Destroy()
{

}

// Gets called when we switch to the SegmentStart controller
void cSegmentStart::Reinit( const flarb_msgs::State &state)
{

}

// Passes reference of "msg", is used as output
// Executes the SegmentStart sub-controller based on the rest of the arguments
enum SUBRETURN cSegmentStart::Execute( tVector &output, const flarb_msgs::State &state, cMap &map)
{
	// First check if there ain't room at both sides
	// TODO check both sides
	tBoundingBox b( tVector( -0.5f, 0), tVector( 0.5f, 0.2f));
	if(map.CheckIntersectionRegion( b))
	{
		return RET_NEXT;
	}

	tVector direction = tVector( 0.0f, 1.0f);
	map.FindFreePath( FLARB_EXTRA_RADIUS, direction, output, true);

	if(output.Length() > 0.4f)
		output.setLength( 0.4f);

	return RET_SUCCESS;
}
