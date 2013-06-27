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
	_StartDistance = state.response.distance;
	_GoalDistance  = state.response.distance + _ParamOffset;
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

	/*********************************
	 * Version 2
	 *********************************/
	if( _ParamFollowVersionTwo)
	{
		// The area
		tBoundingBox boxLeft ( tVector( 0.0f, 0.0f), tVector( -0.5f, 0.7f));
		tBoundingBox boxRight( tVector( 0.0f, 0.0f), tVector( +0.5f, 0.7f));

		//
		float xLeft, xRight;

		bool resLeft  = map.RegionMaximalX( boxLeft,  xLeft);
		bool resRight = map.RegionMinimalX( boxRight, xRight);

		if( !resLeft)
			xLeft  = -0.5f;
		if( !resRight)
			xRight =  0.5f;

		float difference = xRight -  xLeft;
		float offset     = xLeft + (difference / 2);

		if( difference < 0.15f)
		{
			cout << "We are stuck, turning 180 degrees within row" << endl;
			cout << resLeft << ", " << resRight << endl;
			cout << difference << endl;
			cout << "StartDistance " << _StartDistance << endl;
			cout << "PrevGoalDist  " << _GoalDistance << endl;
			cout << "CurrentDist   " << state.response.distance << endl;
			_GoalDistance = (state.response.distance - _StartDistance) + state.response.distance - _ParamDecBlocked;
			cout << "CurGoalDist  " << _GoalDistance << endl;
			return RET_BLOCKED;
		}

		// TODO Steer stronger?
		tVector direction = tVector( offset, _ParamSpeed);
		direction.setLength( _ParamSpeed);

		// Even wanted?
		if( _ParamFollowVTRecheck)
			map.FindFreePath( _ParamRadius, direction, output, true);

		//
		return RET_SUCCESS;

	}
	/*********************************
	 * Version 1
	 *********************************/
	else
	{
		// Just drive straight ahead
		tVector direction = tVector( 0.0f, _ParamSpeed);

		// Attempt to find a path with an extra big radius
		float result = map.FindFreePath( FLARB_EXTRA_RADIUS + _ParamRadius, direction, output, true);

		// Try it again but with the standard car-radius
		// TODO this is getting quite crappy..
		if( result < 0.7)
			result = map.FindFreePath( FLARB_EXTRA_RADIUS, direction, output, true);

		// Attempt again but with false, note that this might not return a solution..
		if( result < 0.3)
			result = map.FindFreePath( FLARB_EXTRA_RADIUS, direction, output, false);

		// Are we really stuck?
		if( result < 0.3)
		{
			cout << "We are stuck, turning 180 degrees within row" << endl;
			cout << "StartDistance " << _StartDistance << endl;
			cout << "PrevGoalDist  " << _GoalDistance << endl;
			cout << "CurrentDist   " << state.response.distance << endl;
			_GoalDistance = (state.response.distance - _StartDistance) + state.response.distance - _ParamDecBlocked;
			cout << "CurGoalDist  " << _GoalDistance << endl;
			return RET_BLOCKED;
		}
		else
		{
			// Add a correction vector
	//#if FLARB_FOLLOW_APPLY_COR_X
	//		result.setX( result.getX() * FLARB_FOLLOW_CORRECT_X);
	//#endif
			return RET_SUCCESS;
		}
	}
}
