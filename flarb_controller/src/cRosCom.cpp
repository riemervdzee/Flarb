// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"
#include "flarb_controller/cRosCom.h"
#include "flarb_controller/cImage.h"
#include "flarb_mapbuilder/MapImage.h"
using namespace std;


int cRosCom::Create( ros::NodeHandle *rosNode)
{
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
	// Pass the message to cImage,
	cImage image( &msg);
	
	image.CountBlockedRectangle( 50, 50, 300, 300);
}
