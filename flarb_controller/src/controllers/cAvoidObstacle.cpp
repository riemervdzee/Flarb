// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "flarb_controller/controllers/cAvoidObstacle.h"
using namespace std;

// Functions executed at the beginning and end of the Application
bool cAvoidObstacle::Create()
{
	return true;
}

void cAvoidObstacle::Destroy()
{

}

// Gets called when we switch to the AvoidObstacle controller
void cAvoidObstacle::Reinit( const flarb_msgs::State &state)
{
	// If this is the first time the sub-controller gets executed in a series
	// Set the _count to a constant
	// TODO we need the axisZ to be quite reliable here..
	_StopCount = 30 * 3;
	_direction = state.response.axisZ + M_PI;

	// 0 <= _direction <= 2xPI
	if( _direction > (2*M_PI))
		_direction -= (2*M_PI);
}

// Passes reference of "msg", is used as output
// Executes the AvoidObstacle sub-controller based on the rest of the arguments
enum SUBRETURN cAvoidObstacle::Execute( tVector &output, const flarb_msgs::State &state, cMap &map)
{
	// As long as _StopCount > 0, try to stop
	if( _StopCount > 0)
	{
		_StopCount--;
		output = tVector();
		return RET_SUCCESS;
	}
	else
	{
		// Check if we are near our goal
		if( state.response.axisZ > (_direction - 0.005) && state.response.axisZ < (_direction + 0.005))
		{
			output = tVector();
			return RET_NEXT;
		}
		else
		{
			// Here is where we turn badly
			output = tVector(0.3f, 0);
			return RET_SUCCESS;
		}
	}
	
	// To get rid of the warnings..
	cout << "[ERROR] cAvoidObstacle, we should never reach this!" << endl;
	return RET_SUCCESS;
}

