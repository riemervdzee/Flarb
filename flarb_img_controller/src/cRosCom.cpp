// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"
#include "flarb_img_controller/cRosCom.h"
#include "flarb_img_controller/cImage.h"
#include "flarb_img_controller/cController.h"

#include "flarb_img_controller/WaypointVector.h"
#include "flarb_img_mapbuilder/MapImage.h"
using namespace std;


int cRosCom::Create( ros::NodeHandle *rosNode, cController *controller)
{
	// Set controller
	_controller = controller;

	// Subscribe to map
	_subMap = rosNode->subscribe<flarb_img_mapbuilder::MapImage>( "map", 1, &cRosCom::ImgCallback, this);

	// We publish to steering/waypoint
	_pubVector = rosNode->advertise<flarb_img_controller::WaypointVector>( "steering/waypoint", 1);

	return 0;
}

int cRosCom::Destroy()
{
	return 0;
}

void cRosCom::PublishWaypoint( const flarb_img_controller::WaypointVector &msg)
{
	_pubVector.publish( msg);
}

void cRosCom::ImgCallback( const flarb_img_mapbuilder::MapImage msg)
{
	// Pass the message to cImage, pass that one back to the main-controller
	cImage image( &msg);
	_controller->CallbackMap( image);
}
