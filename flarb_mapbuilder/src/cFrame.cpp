// Include order: cppstd, ROS, Boost, own-module includes
#include <cmath>
#include <iostream>

#include "ros/ros.h"

#include "flarb_mapbuilder/cFrame.h"
using namespace std;

/*
 * Standard info
 * min -2.35619
 * max 2.35619
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
	vector<float>::const_iterator itr;
	float angle = msg.angle_max + M_PI / 4;

	// Resize if it is the first time, clear when it is being reused
	if( _dataPoints.size() == 0)
	{
		_dataPoints.resize( INIT_SIZE_POINTS);
		_dataObjects.resize( INIT_SIZE_OBJECTS);
	}
	else
	{
		_dataPoints.clear();
		_dataObjects.clear();
	}

	// Go through all range points
	for( itr = msg.ranges.begin(); itr != msg.ranges.end(); itr++, angle -= msg.angle_increment)
	{
		// Temps
		const float range = *(itr);
		sPoint p;

		// Is the value out of range? continue
		//if( range < msg.range_min || range > msg.range_max)
		//	continue;

		// Convert polar coordinates to cartisian
		// TODO, get these cos/sin into a lookup table? performance
		// TODO use orientation to get the right x/y
		p.x = cos( angle) * range;
		p.y = sin( angle) * range;


		// Check if the new point is far away enough from the previous one (if exists at all)
		if( _dataPoints.empty() || 
	      ( sqrt(_dataPoints.back().x - p.x) + sqrt(_dataPoints.back().y - p.y))  > MAX_POINT_DISTANCE)
		{
			// Create a new sObject, and push it
			sObject obj;
			obj.index = _dataPoints.size();
			obj.size = 1;
			_dataObjects.push_back( obj);
		}
		else
		{
			// Increase size of the lastObj
			_dataObjects.back().size++;
		}
		
		// Push the new point
		_dataPoints.push_back( p);
	}
}

