#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include "ros/ros.h"

#include "std_msgs/String.h"
#include "flarb_msgs/WaypointVector.h"
#include "flarb_controller/cMap.h"
#include "flarb_mapbuilder/MapList.h"


// forward declaration of cController
class cController;


/*
 * Handles all communication with ROS
 */
class cRosCom
{
public:
	// Functions executed at the beginning and end of the Application
	int Create( ros::NodeHandle *rosNode, cController *controller);
	int Destroy();

	// Publishes a WaypointVector
	void PublishWaypoint( const flarb_msgs::WaypointVector &msg);

private:
	// We received a map
	void MapCallback( const flarb_mapbuilder::MapList msg);

	// We received orders from the smartphone
	void SmartphoneCallback( const std_msgs::String msg);


	// Pointer to the cController class _NOTE: WE ARE NOT THE OWNER_
	cController *_controller;

	// MapList
	cMap _map;

	// Our subscribers and publishers
	ros::Subscriber _subMap;         //  "/map"
	ros::Subscriber _subSmartphone;  //  "/smartphone/input"
	ros::Publisher  _pubVector;      //  "/steering/waypoint/"
};

#endif // CLASS_FRAME_H

