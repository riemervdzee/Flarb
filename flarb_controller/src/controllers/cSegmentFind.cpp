// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include "ros/ros.h"
#include "flarb_controller/controllers/cSegmentFind.h"

// Functions executed at the beginning and end of the Application
bool cSegmentFind::Create()
{
	return true;
}

void cSegmentFind::Destroy()
{

}

// Gets called when we switch to the SegmentFind controller
void cSegmentFind::Reinit( const flarb_msgs::State &state, const cInputString &str)
{
	_direction = state.response.axisZ;

	// 0 <= _direction <= 2xPI
	if( _direction > (2*M_PI))
		_direction -= (2*M_PI);
	else if( _direction < 0)
		_direction += (2*M_PI);
}

// Passes reference of "msg", is used as output
// Executes the SegmentFind sub-controller based on the rest of the arguments
enum SUBRETURN cSegmentFind::Execute( tVector &output, const flarb_msgs::State &state, cMap &map)
{
	output = tVector();
	return RET_SUCCESS;
}
