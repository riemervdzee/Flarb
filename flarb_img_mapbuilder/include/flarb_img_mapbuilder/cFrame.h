#ifndef CLASS_FRAME_H
#define CLASS_FRAME_H

#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


// Init size of amount of points and objects (avoids reallocations)
#define INIT_SIZE_POINTS  1081   // 1081 is the maximum amount of points we can receive


/*
 * Helper struct
 */
struct sPoint {
	float x;
	float y;
};


/*
 * 
 */
class cFrame
{
public:
	// Generates a frame from a LaserScan message
	void GenerateFrame( const sensor_msgs::LaserScan &msg);

	// Our result, in public domain for easy access
	std::vector<sPoint>  _dataPoints;
};

#endif // CLASS_FRAME_H

