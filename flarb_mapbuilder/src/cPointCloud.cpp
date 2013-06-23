// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_mapbuilder/cPointCloud.h"
using namespace std;


// Functions executed at the beginning and end of the Application
int cPointCloud::Create( ros::NodeHandle *rosNode)
{
	// Reserve obj
	_objects.reserve( OBJECT_AMOUNT_RESERVE);

	// Pass rosnode
	_frame.Create( rosNode);

	return 0;
}
int cPointCloud::Destroy() { return 0;}


// Processes a LaserScan message
void cPointCloud::ProcessMessage( const sensor_msgs::LaserScan &msg)
{
	// First pass the message to the current Frame
	_frame.GenerateFrame( msg);

	// TODO this is a very basic algorithm..
	// Clear the object list, get first datapoint = first object
	_objects.clear();
	sObject obj = { .index = 0, .length = 1};
	_objects.push_back( obj);

	tVector previousVector = _frame._dataPoints[0];
	int previousObject = 0;


	// Go through all remaining datapoints, and see whether they are a new obj or together with the previous
	for( unsigned int i = 1; i < _frame._dataPoints.size(); i++)
	{
		float lengthSquared = (previousVector - _frame._dataPoints[i]).LengthSquared();
		previousVector = _frame._dataPoints[i];

		if( lengthSquared < OBJECT_DISTANCE_MAX)
		{
			_objects[previousObject].length++;
		}
		else
		{
			sObject obj = { .index = i, .length = 1};
			_objects.push_back( obj);
			previousObject++;
		}
	}
}


// Getters for both vectors
const std::vector<tVector>& cPointCloud::getVectorPoints() const
{
	return _frame._dataPoints;
}

const std::vector<sObject>& cPointCloud::getVectorObjects() const
{
	return _objects;
}

