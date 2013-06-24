#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include "ros/ros.h"

#include "std_msgs/String.h"
#include "flarb_msgs/WaypointVector.h"
#include "flarb_msgs/MapList.h"
#include "flarb_msgs/VDState.h"
#include "flarb_controller/cMap.h"


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

	// Publishes a PlantQualityRequest
	void PublishPlantQualityRequest( bool left, bool right);

	// Calls the VD service and returns the response
	void GetVDState( flarb_msgs::VDState &data);

private:
	// We received a map
	void MapCallback( const flarb_msgs::MapList msg);

	// We received orders from the smartphone
	void SmartphoneCallback( const std_msgs::String msg);


	// Pointer to the cController class _NOTE: WE ARE NOT THE OWNER_
	cController *_controller;

	// MapList
	cMap _map;

	// Our subscribers and publishers
	ros::Subscriber _subMap;         //  "/map"
	ros::Subscriber _subSmartphone;  //  "/smartphone/input"
	ros::Publisher  _pubVector;      //  "/steering/waypoint"
	ros::Publisher  _pubPQRequest;   //  "/plantquality/request"

	// The vdmixer service client
	ros::ServiceClient _vdmixer;
};

#endif // CLASS_FRAME_H

