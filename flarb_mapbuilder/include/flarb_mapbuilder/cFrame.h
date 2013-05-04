#ifndef CLASS_FRAME_H
#define CLASS_FRAME_H

#include <vector>

#include "flarb_mapbuilder/types/tMatrix.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define CALCU_COSSIN 0
#define CACHE_COSSIN 0
#define USE_MATRIX   1

#if (CALCU_COSSIN + CACHE_COSSIN + USE_MATRIX) != 1
#error Only select one of the following: CALCU_COSSIN, CACHE_COSSIN or USE_MATRIX!
#endif

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

private:
#if CACHE_COSSIN
	// Caching sin/cos calculations
	static void PreCache( unsigned int size, float angle, float AngleIncrement);
	static std::vector<tMatrix> _cartesian_cache;
#endif
};

#endif // CLASS_FRAME_H

