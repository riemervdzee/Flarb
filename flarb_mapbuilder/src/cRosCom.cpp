// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"
#include "flarb_mapbuilder/Map.h"

#include "flarb_mapbuilder/cRosCom.h"
#include "flarb_mapbuilder/cPointCloud.h"
using namespace std;


int cRosCom::Create( ros::NodeHandle *rosNode)
{
	// Init publisher obj
	_pubMap = rosNode->advertise<flarb_mapbuilder::Map>( "map", 1);

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
	// Give the LaserScan message to the PointCloud
	_pointCloud.ProcessMessage( msg);

	// Construct a Map message and publish it
	flarb_mapbuilder::Map mesg;
	_mapbuilder.Build( _pointCloud, mesg);
	_pubMap.publish( mesg);
}
