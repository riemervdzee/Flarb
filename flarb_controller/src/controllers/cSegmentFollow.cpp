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

void cSegmentFollow::Reinit( const flarb_msgs::VDState &state)
{
	_GoalDistance = state.response.distance + FLARB_FOLLOW_OFFSET;
}

// Passes reference of "vector", is used as output
// Executes the SegmentFollow sub-controller based on the rest of the arguments
enum SUBRETURN cSegmentFollow::Execute( tVector &output, const flarb_msgs::VDState &state, cMap &map)
{
	// After _GoalDistance is crossed, check if we are at the end of the segment
	if( state.response.distance > _GoalDistance)
	{
		tBoundingBox b( tVector( -0.5f, -0.2f), tVector( 0.5f, 0.4f));
		if(!map.CheckIntersectionRegion( b))
		{
			return RET_NEXT;
		}
	}

	// Just drive straight ahead
	tVector direction = tVector( 0.0f, FLARB_FOLLOW_SPEED);

	// Attempt to find a path with an extra big radius
	float result = map.FindFreePath( FLARB_EXTRA_RADIUS + FLARB_FOLLOW_EXTRA, direction, output, true);

	// Try it again but with the standard car-radius
	// TODO this is getting quite crappy..
	if( result < 0.5)
		result = map.FindFreePath( FLARB_EXTRA_RADIUS, direction, output, true);

	// Are we still stuck?
	if( result < 0.1)
		result = map.FindFreePath( FLARB_EXTRA_RADIUS, direction, output, false);

	if( result < 0.1)
		return RET_BLOCKED;
	else
		return RET_SUCCESS;
}
