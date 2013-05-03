#ifndef CLASS_FRAME_H
#define CLASS_FRAME_H

#include <vector>

#include "flarb_mapbuilder/types/tVector.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


// Init size of amount of points and objects (avoids reallocations)
#define INIT_SIZE_POINTS  1082   // 1081 is the maximum amount of points we can receive


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

