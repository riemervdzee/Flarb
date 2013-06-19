// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include "ros/ros.h"
#include "flarb_controller/controllers/cFreeRun.h"
using namespace std;

// Executed at the beginning of the sub-controler
bool cFreeRun::Create()
{
	return true;
}

void cFreeRun::Destroy()
{

}

// Passes reference of "output", which will be the WayPoint if the sub-controller succeeded
// Executes the FreeRun sub-controller based on the rest of the arguments
// State is the VDMixer state, map is the laserscan map, 
// reinit tells whether it is the first time in a series the sub-controller is executed
enum SUBRETURN cFreeRun::Execute( tVector &output, 
		const flarb_msgs::State &state, cMap &map, bool reinit)
{
	// Try to drive straight ahead
	tVector direction = tVector( 0.0f, 1.0f);
	float result = map.FindFreePath( FLARB_EXTRA_RADIUS, direction, output, false);

	if( result > 0.25f)
	{
		if(output.Length() > 0.5f)
			output.setLength( 0.5f);
		return RET_SUCCESS;
	}

	// Try to drive Right
	direction = tVector( 1.0f, 0.0f);
	result = map.FindFreePath( FLARB_EXTRA_RADIUS, direction, output, false);

	if( result > 0.25f)
	{
		if(output.Length() > 0.5f)
			output.setLength( 0.5f);
		return RET_SUCCESS;
	}

	// Try to drive Left
	direction = tVector( -1.0f, 0.0f);
	result = map.FindFreePath( FLARB_EXTRA_RADIUS, direction, output, false);

	if( result > 0.25f)
	{
		if(output.Length() > 0.5f)
			output.setLength( 0.5f);
		return RET_SUCCESS;
	}

	// We failed
	return RET_BLOCKED;
}
