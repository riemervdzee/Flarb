#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "flarb_simulation/cCar.h"
#include "flarb_simulation/cMap.h"

// forward declaration of cController
class cController;


/*
 * Handles all communication with ROS
 */
class cRosCom
{
public:
	// Functions executed at the beginning and end of the Application
	int Create( ros::NodeHandle *rosNode, cController *controller, int hz);
	int Destroy();

	// Publishes a WaypointVector
	void PublishLaserScan( const cCar &car, const cMap &map);

private:
	// We received a map
	//void MapbuildCallback( const flarb_mapbuilder::MapList msg);


	//
	sensor_msgs::LaserScan _msg;

	// Pointer to the cController class _NOTE: WE ARE NOT THE OWNER_
	cController *_controller;

	// Subscribers and publishers
	ros::Publisher _pubSick;   //  "/sick/scan_filtered/"
	//ros::Subscriber _subMap;
};

#endif // CLASS_FRAME_H

