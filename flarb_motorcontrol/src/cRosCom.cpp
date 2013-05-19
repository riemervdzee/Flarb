// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_motorcontrol/cRosCom.h"
#include "flarb_motorcontrol/cController.h"
using namespace std;

// Helper struct
union int2char{
	char buff[4];
	int  val;
};


int cRosCom::Create( ros::NodeHandle *rosNode, cController *controller)
{
	// Init publisher obj
	_pubCanbus = rosNode->advertise<flarb_canbus::CanMessage>( "/canbus/send", 1);

	// Init Subscriber obj.
	_subWaypoint = rosNode->subscribe<flarb_controller::WaypointVector>( "/steering/waypoint", 1, &cRosCom::WVCallback, this);

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
void cRosCom::SendMotorStrength( int l, int r)
{
	flarb_canbus::CanMessage msg;
	msg.identifier = CANBUS_ID_OUTPUT;
	msg.data.reserve( 8);

	int2char _l, _r;
	_l.val = l;
	msg.data[0] = _l.buff[0];
	msg.data[1] = _l.buff[1];
	msg.data[2] = _l.buff[2];
	msg.data[3] = _l.buff[3];

	_r.val = r;
	msg.data[4] = _r.buff[0];
	msg.data[5] = _r.buff[1];
	msg.data[6] = _r.buff[2];
	msg.data[7] = _r.buff[3];

	_pubCanbus.publish( msg);
}


/*
 * Passes the Waypoint back to the controller for processing
 */
void cRosCom::WVCallback( const flarb_controller::WaypointVector msg)
{
	_controller->SetWaypoint( msg.x, msg.y);
}

