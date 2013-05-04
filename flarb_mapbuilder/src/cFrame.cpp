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
 *
 */

/*
 *
 */
void cFrame::GenerateFrame( const sensor_msgs::LaserScan &msg)
{
#if CACHE_COSSIN
	// If the cartesian_cache is still empty, fill it
	if( _cartesian_cache.size() == 0)
		PreCache( msg.ranges.size(), msg.angle_min, msg.angle_increment);

#elif CALCU_COSSIN
	// Vars
	float angle = msg.angle_min;

#elif USE_MATRIX
	// Vars
	tMatrix angle( msg.angle_min);
	tMatrix increment( msg.angle_increment);

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
			continue;

		// Convert polar coordinates to cartesian, and add it
		// TODO use inclination/gyro to get rid of any ground info, plus correct X/Y
#if CACHE_COSSIN
		tVector p = _cartesian_cache[i].getVector( range);

#elif CALCU_COSSIN
		tVector p( cos( angle) * range, sin( angle) * range);
		angle += msg.angle_increment;

#elif USE_MATRIX
		tVector p = angle.getVector( range);
		angle *= increment;
#endif

		_dataPoints.push_back( p);
	}
}

#if CACHE_COSSIN
/*
 *
 */
std::vector<tMatrix> cFrame::_cartesian_cache;
void cFrame::PreCache( unsigned int size, float angle, float AngleIncrement)
{
	_cartesian_cache.reserve( size);

	for( unsigned int i = 0; i < size; i++, angle += AngleIncrement)
	{
		tMatrix mat( angle);
		_cartesian_cache.push_back( mat);
	}
}
#endif

