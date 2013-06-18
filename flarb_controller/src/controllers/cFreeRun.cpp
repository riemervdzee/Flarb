// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include "ros/ros.h"
#include "flarb_controller/controllers/cFreeRun.h"
using namespace std;

// Functions executed at the beginning and end of the Application
bool cFreeRun::Create()
{
	return true;
}

void cFreeRun::Destroy()
{

}

// Passes reference of "vector", is used as output
// Executes the FollowSegment sub-controller based on the rest of the arguments
// TODO maybe more parameters?
enum SUBRETURN cFreeRun::Execute( tVector &output, 
		const flarb_VDMixer::State &state, cMap &map, bool reinit)
{
	// Just continue the current row
	tVector direction = tVector( 0.0f, 0.5f);
	float result = map.FindFreePath( FLARB_EXTRA_RADIUS, direction, output, false);

	// TODO here we can do stuff with the result, whether 0 or negative 
	// return false maybe? For now, just set vector = output

	return RET_SUCCESS;
}
