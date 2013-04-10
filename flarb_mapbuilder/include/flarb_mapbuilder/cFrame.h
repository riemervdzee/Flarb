#ifndef CLASS_FRAME_H
#define CLASS_FRAME_H

#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


// Init size of amount of points and objects (avoids reallocations)
#define INIT_SIZE_POINTS  1081   // 1081 is the maximum amount of points we can receive
#define INIT_SIZE_OBJECTS  100

// Maximum distance between two points, before they are considered two separate objects
// NOTE this is in squared distance centimeters
#define MAX_POINT_DISTANCE (0.10 * 0.10)


/*
 * Helper structs
 */
struct sPoint {
	float x;
	float y;
};

struct sObject {
	int index;   // Index of the first point in the big array
	int size;    // How many points this object has
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
	std::vector<sObject> _dataObjects;
};

#endif // CLASS_FRAME_H

