#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include "ros/ros.h"

#include "flarb_canbus/DualMotorSpeed.h"
#include "flarb_canbus/DualMotorEncoder.h"
#include "flarb_controller/WaypointVector.h"


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
	void SendMotorStrength( int l, int r, bool brake);

private:
	// We received a waypoint vector
	void WVCallback( const flarb_controller::WaypointVector msg);
	void EncoderCallback( const flarb_canbus::DualMotorEncoder msg);


	// Our subscribers and publishers
	ros::Subscriber _subWaypoint;  //  "/steering/waypoint"
	ros::Subscriber _subEncoder;   //  "/canbus/encoder"
	ros::Publisher  _pubSpeed;     //  "/canbus/send"
	ros::Publisher  _pubEncoder;   //  "/steering/encoder"

	// When getting a waypoint or encoder info, we need to call the controller back
	cController *_controller;
};

#endif // CLASS_ROSCOM_H
