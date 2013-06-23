// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"
#include "flarb_msgs/MapList.h"

#include "flarb_mapbuilder/cRosCom.h"
#include "flarb_mapbuilder/cPointCloud.h"
using namespace std;


int cRosCom::Create( ros::NodeHandle *rosNode)
{
	// Create a private nodehandle, pass it to pointCloud (for parameters)
	ros::NodeHandle n("~");
	_pointCloud.Create( &n);

	// Init publisher obj
	_pubMap = rosNode->advertise<flarb_msgs::MapList>( "map", 1);

	// Init Subscriber obj.
	_subSicklaser = rosNode->subscribe<sensor_msgs::LaserScan>( "/sick/scan_filtered", 1, &cRosCom::ScanCallback, this);

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
	flarb_msgs::MapList mesg;
	_mapbuilder.Build( _pointCloud, mesg);
	_pubMap.publish( mesg);
}
