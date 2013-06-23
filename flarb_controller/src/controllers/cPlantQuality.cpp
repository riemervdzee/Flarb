// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include "ros/ros.h"
#include "flarb_controller/controllers/cPlantQuality.h"

// Functions executed at the beginning and end of the Application
bool cPlantQuality::Create()
{
	return true;
}

void cPlantQuality::Destroy()
{

}

// Gets called when we switch to the SegmentFollow controller
void cPlantQuality::Reinit( const flarb_msgs::VDState &state)
{

}

// This is a stripped Execute function, as we only need to know the surrounding
// The sub-controller doesn't influence the surrounding at all..
void cPlantQuality::Execute( const cRosCom &_roscom, cMap &map)
{
	// TODO everything!
}
