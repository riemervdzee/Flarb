// Include order: cppstd, ROS, Boost, own-module includes
#include <cmath>
#include <iostream>

#include "ros/ros.h"

#include "flarb_mapbuilder/cFrame.h"
using namespace std;

/*
 * Standard info
 * min angle -0.785398
 * max angle  3.92699
 * angl. incr 0.00436332
 * time_increment 3.70028e-05
 * scan_time 0.04
 * n 1081
 * amount of points 721
 * amount of obj    336
 *
 */

/*
 *
 */
void cFrame::GenerateFrame( const sensor_msgs::LaserScan &msg)
{
	// Vars
	float angle = msg.angle_min;

	// Resize if it is the first time, clear when it is being reused
	if( _dataPoints.size() == 0)
		_dataPoints.resize( INIT_SIZE_POINTS);
	else
		_dataPoints.clear();


	// Go through all range points
	for( unsigned int i = 0; i < msg.ranges.size(); i++, angle += msg.angle_increment)
	{
		// Temps
		const float range = msg.ranges.at( i);
		sPoint p;

		// Is the value out of range? continue
		if( range < msg.range_min || range > msg.range_max)
			continue;

		// Convert polar coordinates to cartisian
		// TODO, get these cos/sin into a lookup table? performance
		// TODO use orientation to get the right x/y
		p.x = cos( angle) * range;
		p.y = sin( angle) * range;
		
		// Push the new point
		_dataPoints.push_back( p);
	}
}

