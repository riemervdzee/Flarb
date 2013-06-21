// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_msgs/Encoder.h"
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



int cRosCom::Create( cController *controller, ros::NodeHandle *rosNode)
{
	// Set ref
	_controller = controller;

	// Init Subscribers and publishers
	_subWaypoint = rosNode->subscribe<flarb_msgs::WaypointVector>  ( "/steering/waypoint", 1, &cRosCom::WVCallback, this);
	_subEncoder  = rosNode->subscribe<flarb_msgs::DualMotorEncoder>( "/canbus/encoder", 1, &cRosCom::EncoderCallback, this);
	_pubSpeed    = rosNode->advertise<flarb_msgs::DualMotorSpeed>  ( "/canbus/speed", 1);
	_pubEncoder  = rosNode->advertise<flarb_msgs::Encoder>         ( "/steering/encoder", 1);

	return 0;
}

int cRosCom::Destroy()
{
	return 0;
}


/*
 * Tells whether to brake, to be called by the controller
 */
void cRosCom::MotorBrake()
{
	// Canbus message
	flarb_msgs::DualMotorSpeed msg;
	msg.speed_right = 0;
	msg.speed_left  = 0;

	// Set both brake flags
	msg.flags = 0;
	msg.flags |= (1 << 0);
	msg.flags |= (1 << 1);

	_pubSpeed.publish( msg);
}


/*
 * Passes the Waypoint back to the controller for processing
 */
void cRosCom::WVCallback( const flarb_msgs::WaypointVector msg)
{
	// Tell the controller we received something
	_controller->WaypointReceived();

	// Msg X/Y
	float x = msg.x;
	float y = msg.y;

	// resize to VECTOR_MAXSEC
	// TODO buggy
	/*float size = (x*x + y*y);
	if(size > (VECTOR_MAXSEC*VECTOR_MAXSEC)) // if size is bigger than 0.5^2
	{
		size = sqrt( size);
		float L = VECTOR_MAXSEC / size;
		x *= L;
		y *= L;
	}*/

	// Calculate
	int _inputRight, _inputLeft;

	// Should we turn around the axle?
	if( msg.QuickTurn)
	{
		float SpeedFactor = sqrt(x*x) * VECTOR2MOTOR;

		if( x > 0)
			_inputRight = (int)-SpeedFactor;
		else
			_inputRight = (int)+SpeedFactor;

		_inputLeft = -_inputRight;
	}
	else
	{
		float AlphaRadians = atan2( y, x);
		float SpeedFactor  = sqrt(x*x + y*y*4) * VECTOR2MOTOR;
		float R            = AlphaRadians / M_PI;
		float L            = (AlphaRadians < 0) ? (-1 - R) : (1-R);
		_inputRight        = (int)(R * SpeedFactor);
		_inputLeft         = (int)(L * SpeedFactor);
	}

	// Canbus message
	flarb_msgs::DualMotorSpeed motormsg;
	motormsg.speed_right = (int16_t) _inputRight;
	motormsg.speed_left  = (int16_t) _inputLeft;
	motormsg.flags       = 0;

	_pubSpeed.publish( motormsg);

#if 0
	cout << "[SET] Right " << _inputRight << ", Left " << _inputLeft << endl;
#endif
}

void cRosCom::EncoderCallback( const flarb_msgs::DualMotorEncoder msg)
{
	flarb_msgs::Encoder newMsg;

	newMsg.speed_left  = msg.speed_left  * PULSE2METER;
	newMsg.speed_right = msg.speed_right * PULSE2METER;

	_pubEncoder.publish( newMsg);
}

