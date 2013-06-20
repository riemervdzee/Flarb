// Include order: cppstd, ROS, Boost, own-module includes
#include <cmath>
#include <iostream>

#include "ros/ros.h"

#include "flarb_mapbuilder/cFrame.h"
using namespace std;

#define SICK_LASER_UPSIDEDOWN 1

#define CALCU_COSSIN 1
#define USE_MATRIX   0

#if (CALCU_COSSIN + USE_MATRIX) != 1
#error Only select one of the following: CALCU_COSSIN or USE_MATRIX!
#endif

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
#if SICK_LASER_UPSIDEDOWN
	#if CALCU_COSSIN
		// Vars
		float angle = msg.angle_max;

	#elif USE_MATRIX
		// Vars
		tMatrix angle( msg.angle_max);
		tMatrix increment( -msg.angle_increment);

	#endif
#else
	#if CALCU_COSSIN
		// Vars
		float angle = msg.angle_min;

	#elif USE_MATRIX
		// Vars
		tMatrix angle( msg.angle_min);
		tMatrix increment( msg.angle_increment);

	#endif
#endif

	// Reserve space for datapoints if it is the first time for this cFrame obj
	// Clear when it is being reused
	if( _dataPoints.size() == 0)
		_dataPoints.reserve( msg.ranges.size());
	else
		_dataPoints.clear();


	// Go through all range points
	for( unsigned int i = 0; i < msg.ranges.size(); i++)
	{
		// Get the range
		const float range = msg.ranges.at( i);

		// Is the value out of range? continue
		if( range < msg.range_min || range > msg.range_max)
		{
#if CALCU_COSSIN
	#if SICK_LASER_UPSIDEDOWN
		angle -= msg.angle_increment;
	#else
		angle += msg.angle_increment;
	#endif

#elif USE_MATRIX
		angle *= increment;
#endif
			continue;
		}

		// Convert polar coordinates to cartesian, and add it
		// TODO use inclination/gyro to get rid of any ground info, plus correct X/Y

#if CALCU_COSSIN
		tVector p( cos( angle) * range, sin( angle) * range);

	#if SICK_LASER_UPSIDEDOWN
		angle -= msg.angle_increment;
	#else
		angle += msg.angle_increment;
	#endif

#elif USE_MATRIX
		tVector p = angle.getVector( range);
		angle *= increment;
#endif

		_dataPoints.push_back( p);
	}
}

