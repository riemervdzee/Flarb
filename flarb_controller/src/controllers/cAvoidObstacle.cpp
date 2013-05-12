// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include "ros/ros.h"
#include "flarb_controller/controllers/cAvoidObstacle.h"

// Functions executed at the beginning and end of the Application
bool cAvoidObstacle::Create()
{
	return true;
}

void cAvoidObstacle::Destroy()
{

}

// Passes reference of "msg", is used as output
// Executes the AvoidObstacle sub-controller based on the rest of the arguments
// TODO maybe more parameters?
bool cAvoidObstacle::Execute( tVector &vector, const flarb_mapbuilder::Map &map)
{
	return false;
}

