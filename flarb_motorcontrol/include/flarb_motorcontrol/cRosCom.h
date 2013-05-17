#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include "ros/ros.h"

#include "flarb_canbus/CanMessage.h"
#include "flarb_controller/WaypointVector.h"


/*
 * 
 */
class cRosCom
{
public:
	// Functions executed at the beginning and end of the Application
	int Create( ros::NodeHandle *rosNode);
	int Destroy();

private:
	// We received a waypoint vector
	void WVCallback( const flarb_controller::WaypointVector msg);


	// Subscribing to "/steering/waypoint"
	ros::Subscriber _subWaypoint;

	// We send to "/canbus/send"
	ros::Publisher _pubCanbus;
};

#endif // CLASS_ROSCOM_H
