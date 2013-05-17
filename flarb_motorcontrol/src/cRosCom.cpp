// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_motorcontrol/cRosCom.h"
using namespace std;


int cRosCom::Create( ros::NodeHandle *rosNode)
{
	// Init publisher obj
	_pubCanbus = rosNode->advertise<flarb_canbus::CanMessage>( "/canbus/send", 1);

	// Init Subscriber obj.
	_subWaypoint = rosNode->subscribe<flarb_controller::WaypointVector>( "/steering/waypoint", 1, &cRosCom::WVCallback, this);

	return 0;
}

int cRosCom::Destroy()
{
	return 0;
}

void cRosCom::WVCallback( const flarb_controller::WaypointVector)
{
	
}
