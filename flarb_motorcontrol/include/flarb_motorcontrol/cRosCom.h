#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include "ros/ros.h"

#include "flarb_canbus/CanMessage.h"
#include "flarb_controller/WaypointVector.h"

#define CANBUS_ID_OUTPUT  0x11
#define CANBUS_ID_INPUT   0x12

// Class prototype
class cController;

/*
 * 
 */
class cRosCom
{
public:
	// Functions executed at the beginning and end of the Application
	int Create( ros::NodeHandle *rosNode, cController *controller);
	int Destroy();

	// Send motor strengths on the canbus
	void SendMotorStrength( int l, int r);

private:
	// We received a waypoint vector
	void WVCallback( const flarb_controller::WaypointVector msg);


	// Subscribing to "/steering/waypoint"
	ros::Subscriber _subWaypoint;

	// We send to "/canbus/send"
	ros::Publisher _pubCanbus;

	// When getting a waypoint, call the controller back
	cController *_controller;
};

#endif // CLASS_ROSCOM_H
