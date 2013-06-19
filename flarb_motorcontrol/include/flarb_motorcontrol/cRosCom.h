#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include "ros/ros.h"

#include "flarb_msgs/DualMotorSpeed.h"
#include "flarb_msgs/DualMotorEncoder.h"
#include "flarb_msgs/WaypointVector.h"


// Class prototype
class cController;

/*
 * 
 */
class cRosCom
{
public:
	// Functions executed at the beginning and end of the Application
	int Create( ros::NodeHandle *rosNode);
	int Destroy();

	// Tells whether to brake, to be called by the controller
	void MotorBrake();

private:
	// We received a waypoint vector
	void WVCallback( const flarb_msgs::WaypointVector msg);
	void EncoderCallback( const flarb_msgs::DualMotorEncoder msg);


	// Our subscribers and publishers
	ros::Subscriber _subWaypoint;  //  "/steering/waypoint"
	ros::Subscriber _subEncoder;   //  "/canbus/encoder"
	ros::Publisher  _pubSpeed;     //  "/canbus/send"
	ros::Publisher  _pubEncoder;   //  "/steering/encoder"
};

#endif // CLASS_ROSCOM_H
