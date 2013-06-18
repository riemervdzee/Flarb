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

// Passes reference of "msg", is used as output
// Executes the FindSegment sub-controller based on the rest of the arguments
// TODO maybe more parameters?
void cPlantQuality::Execute( const cRosCom &_roscom, cMap &map);
{
	// TODO everything!
}
