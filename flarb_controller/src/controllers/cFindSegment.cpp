// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_controller/controllers/cFindSegment.h"

// Functions executed at the beginning and end of the Application
bool cFindSegment::Create()
{
	return true;
}

void cFindSegment::Destroy()
{

}

// Passes reference of "msg", is used as output
// Executes the FindSegment sub-controller based on the rest of the arguments
// TODO maybe more parameters?
bool cFindSegment::Execute( tVector &vector, const cImage &image)
{
	return false;
}
