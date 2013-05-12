// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include "ros/ros.h"
#include "flarb_controller/controllers/cFollowSegment.h"
using namespace std;

// Functions executed at the beginning and end of the Application
bool cFollowSegment::Create()
{
	return true;
}

void cFollowSegment::Destroy()
{

}

// Passes reference of "vector", is used as output
// Executes the FollowSegment sub-controller based on the rest of the arguments
// TODO maybe more parameters?
bool cFollowSegment::Execute( tVector &vector, const flarb_mapbuilder::Map &image)
{

	return true;
}
