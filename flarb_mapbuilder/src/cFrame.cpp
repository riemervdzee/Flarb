// Include order: cppstd, ROS, Boost, own-module includes
#include <cstring>
#include <cmath>
#include <iostream>

#include "ros/ros.h"

#include "flarb_mapbuilder/cFrame.h"
using namespace std;

/*
 * Const return-by-reference functions to get both vectors
 * Note, if you use these vectors, make sure you call it with referencing!
 * For speed isues etc. eg: const std::vector<sPoint> &a = frame.getPoints();
 */
const vector<sPoint> &cFrame::getPoints() const
{
	return _dataPoints;
}
const vector<sObject> &cFrame::getObjects() const
{
	return _dataObjects;
}

/*
 *
 */
void cFrame::GenerateFrame( const sensor_msgs::LaserScan &msg)
{
	// Vars
	vector<float>::const_iterator itr;
	float angle = msg.angle_min;

	// Performance counters
	int branch_min = 0, branch_max = 0;

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
	for( itr = msg.ranges.begin(); itr != msg.ranges.end(); itr++, angle += msg.angle_increment)
	{
		// Temps
		float range = *(itr);
		sPoint p;

		// Is the value out of range? continue TODO check which one occurs the most
		if( range < msg.range_min)
		{
			branch_min++;
			continue;
		}
		else if( range > msg.range_max)
		{
			branch_max++;
			continue;
		}

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
	
	// Output some debugging
	cout << "Perf counters " << branch_min << " " << branch_max << endl;
	cout << "amount of points " << _dataPoints.size()  << endl;
	cout << "amount of obj    " << _dataObjects.size() << endl << endl << endl;
}

