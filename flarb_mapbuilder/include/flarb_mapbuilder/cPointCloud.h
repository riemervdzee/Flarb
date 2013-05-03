#ifndef CLASS_POINTCLOUD_H
#define CLASS_POINTCLOUD_H

#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "flarb_mapbuilder/types/tVector.h"
#include "flarb_mapbuilder/cFrame.h"


// Maximum distance between points before considered separate objects 
// This is in meters, squared
#define OBJECT_DISTANCE_MAX (0.10f * 0.10f)

// The amount of objects we reserve, note it autogrows if exceeds
#define OBJECT_AMOUNT_RESERVE 300


/*
 * Helper struct, stores the index of the points it owns, plus the amount
 */
struct sObject {
	unsigned int index;
	unsigned int length;
};


/*
 * 
 */
class cPointCloud
{
public:
	// Functions executed at the beginning and end of the Application
	int Create();
	int Destroy();

	// Processes a LaserScan message
	void ProcessMessage( const sensor_msgs::LaserScan &msg);

	// Getters for both vectors
	const std::vector<tVector>& getVectorPoints() const;
	const std::vector<sObject>& getVectorObjects() const;

private:

	// TODO note, we got no vector for the Points yet. 
	// we just base the PointCloud on the last added frame only
	std::vector<sObject> _objects;
	cFrame _frame;
};

#endif // CLASS_POINTCLOUD_H

