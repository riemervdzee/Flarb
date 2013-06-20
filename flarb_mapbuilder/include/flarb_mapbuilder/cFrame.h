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
	// Generates a frame from a LaserScan message
	void GenerateFrame( const sensor_msgs::LaserScan &msg);

	// Our result, in public domain for easy access
	std::vector<tVector> _dataPoints;
};

#endif // CLASS_FRAME_H

