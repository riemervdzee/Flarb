// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"
#include "flarb_controller/cRosCom.h"
#include "flarb_controller/cImage.h"
#include "flarb_controller/cController.h"

#include "flarb_mapbuilder/MapImage.h"
using namespace std;


int cRosCom::Create( ros::NodeHandle *rosNode, cController *controller)
{
	// Set controller
	_controller = controller;

	// Subscribe to map
	_subMap = rosNode->subscribe<flarb_mapbuilder::MapImage>( "map", 1, &cRosCom::ImgCallback, this);

	return 0;
}

int cRosCom::Destroy()
{
	return 0;
}

void cRosCom::ImgCallback( const flarb_mapbuilder::MapImage msg)
{
	// Pass the message to cImage, pass that one back to the main-controller
	cImage image( &msg);
	_controller->CallbackMap( image);
}
