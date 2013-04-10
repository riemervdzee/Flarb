// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"
#include "flarb_mapbuilder/cRosCom.h"
#include "flarb_mapbuilder/MapImage.h"
using namespace std;


int cRosCom::Create( ros::NodeHandle *rosNode)
{
	// Init publisher obj
	_pubMap = rosNode->advertise<flarb_mapbuilder::MapImage>( "map", 1);

	// Init Subscriber obj.
	_subSicklaser = rosNode->subscribe<sensor_msgs::LaserScan>( "/sick/scan", 1, &cRosCom::ScanCallback, this);


	return 0;
}

int cRosCom::Destroy()
{
	return 0;
}

void cRosCom::ScanCallback( const sensor_msgs::LaserScan msg)
{
	// Give the LaserScan message to the Mapbuilder
	_map.ProcessMessage( msg);
	_map.RegenerateImage();

	// TODO send the image
	flarb_mapbuilder::MapImage mesg;
	
	mesg.data = vector<uint8_t>( _map._image._data, _map._image._data + IMAGE_SIZE);
	
	_pubMap.publish( mesg);
}
