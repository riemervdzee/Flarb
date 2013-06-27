// Include order: cppstd, ROS, Boost, own-module includes
#include <cmath>
#include <iostream>

#include "ros/ros.h"

#include "flarb_mapbuilder/cFrame.h"
using namespace std;

//
void cFrame::Create( ros::NodeHandle *rosNode)
{
	rosNode->param<bool>( "UseFullRange", UseFullRange, false);

	rosNode->param<double>( "RangeMin", RangeMin, 0.0);
	rosNode->param<double>( "RangeMax", RangeMax, M_PI);
}

/*
 * Standard info
 * min angle -0.785398
 * max angle  3.92699
 * angl. incr 0.00436332
 * time_increment 3.70028e-05
 * scan_time 0.04
 * n 1081
 *
 */

/*
 *
 */
void cFrame::GenerateFrame( const sensor_msgs::LaserScan &msg)
{
	// Var
	float angle = msg.angle_min;

	// Reserve space for datapoints if it is the first time for this cFrame obj
	// Clear when it is being reused
	if( _dataPoints.size() == 0)
		_dataPoints.reserve( msg.ranges.size());
	else
		_dataPoints.clear();


	// Go through all range points
	for( unsigned int i = 0; i < msg.ranges.size(); i++, angle += msg.angle_increment)
	{
		if( !UseFullRange)
		{
			if( angle < RangeMin)
				continue;
			if( angle > RangeMax)
				continue;
		}

		// Get the range
		const float range = msg.ranges.at( i);

		// Is the value out of range? continue
		if( range < msg.range_min || range > msg.range_max)
			continue;

		// Convert polar coordinates to cartesian, and add it
		// TODO use inclination/gyro to get rid of any ground info, plus correct X/Y
		tVector p( cos( angle) * range, sin( angle) * range);

		_dataPoints.push_back( p);
	}
}

