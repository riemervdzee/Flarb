// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_motorcontrol/cRosCom.h"
#include "flarb_motorcontrol/cController.h"
using namespace std;

/*
 * Helper union to "convert" signed/unsigned ints (16/32) to char buffers
 */
union mix_t {
	int32_t  s32;
	uint32_t u32;

	struct {
		int16_t hi;
		int16_t lo;
	} s16;

	struct {
		uint16_t hi;
		uint16_t lo;
	} u16;

	char c[4];
};



int cRosCom::Create( ros::NodeHandle *rosNode, cController *controller)
{
	// Init Subscribers and publishers
	_subWaypoint = rosNode->subscribe<flarb_controller::WaypointVector>( "/steering/waypoint", 1, &cRosCom::WVCallback, this);
	_subEncoder  = rosNode->subscribe<flarb_canbus::DualMotorEncoder>  ( "/canbus/encoder", 1, &cRosCom::EncoderCallback, this);
	_pubSpeed    = rosNode->advertise<flarb_canbus::DualMotorSpeed>    ( "/canbus/send", 1);


	// Set controller ref
	_controller = controller;

	return 0;
}

int cRosCom::Destroy()
{
	return 0;
}


/*
 * Sends the requested motor strengths via the canbus
 */
void cRosCom::SendMotorStrength( int l, int r, bool brake)
{
	flarb_canbus::DualMotorSpeed msg;
	msg.speed_left  = (int16_t) l;
	msg.speed_right = (int16_t) r;

	_pubSpeed.publish( msg);
}


/*
 * Passes the Waypoint back to the controller for processing
 */
void cRosCom::WVCallback( const flarb_controller::WaypointVector msg)
{
	_controller->SetWaypoint( msg.x, msg.y);
}

void cRosCom::EncoderCallback( const flarb_canbus::DualMotorEncoder msg)
{

}

