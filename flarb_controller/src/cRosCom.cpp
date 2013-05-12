// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"
#include "flarb_controller/WaypointVector.h"
#include "flarb_mapbuilder/Map.h"

#include "flarb_controller/cRosCom.h"
#include "flarb_controller/cController.h"
using namespace std;


int cRosCom::Create( ros::NodeHandle *rosNode, cController *controller)
{
	// Set controller
	_controller = controller;

	// Subscribe to map
	_subMap = rosNode->subscribe<flarb_mapbuilder::Map>( "map", 1, &cRosCom::MapbuildCallback, this);

	// We publish to steering/waypoint
	_pubVector = rosNode->advertise<flarb_controller::WaypointVector>( "steering/waypoint", 1);

	return 0;
}

int cRosCom::Destroy()
{
	return 0;
}

void cRosCom::PublishWaypoint( const flarb_controller::WaypointVector &msg)
{
	_pubVector.publish( msg);
}

void cRosCom::MapbuildCallback( const flarb_mapbuilder::Map msg)
{
	// Pass the message to the main-controller
	_controller->CallbackMap( msg);
}
