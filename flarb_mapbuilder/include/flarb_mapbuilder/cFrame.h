#ifndef CLASS_FRAME_H
#define CLASS_FRAME_H

#include <vector>

#include "flarb_mapbuilder/types/tMatrix.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


/*
 * 
 */
class cFrame
{
public:
	// C-tor
	cFrame() : UseFullRange( true) {}

	// Passes rosnode
	void Create( ros::NodeHandle *rosNode);

	// Generates a frame from a LaserScan message
	void GenerateFrame( const sensor_msgs::LaserScan &msg);

	// Our result, in public domain for easy access
	std::vector<tVector> _dataPoints;

	// Param values
	bool UseFullRange;
	double RangeMin;
	double RangeMax;
};

#endif // CLASS_FRAME_H

