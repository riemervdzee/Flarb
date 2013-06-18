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

// Passes reference of "msg", is used as output
// Executes the FindSegment sub-controller based on the rest of the arguments
// TODO maybe more parameters?
enum SUBRETURN cSegmentFind::Execute( tVector &output, 
		const flarb_VDMixer::State &state, const cMap &map, bool reinit)
{
	return RET_SUCCESS;
}
