// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"
#include "flarb_mapbuilder/cRosCom.h"
using namespace std;


int cRosCom::Create( ros::NodeHandle *rosNode)
{
	// Init publisher obj
	//_pubMap = rosNode->advertise<flarb_canbus::CanMessage>( "map", 10);

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
	// output some debugging info
	cout << "min "            << msg.angle_min        << endl <<
			"max "            << msg.angle_max        << endl <<
			"angl. incr "     << msg.angle_increment  << endl <<
			"time_increment " << msg.time_increment   << endl <<
			"scan_time "      << msg.scan_time        << endl <<
			"n "              << msg.ranges.size()    << endl;

	// Give the LaserScan message to the _frame
	_frame.GenerateFrame( msg);

	// TODO blend the cFrame obj into the current build map
	// Generate map image
}
