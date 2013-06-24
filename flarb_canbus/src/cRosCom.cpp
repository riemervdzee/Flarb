#include <algorithm>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_canbus/cRosCom.h"
#include "flarb_canbus/cCanbus.h"
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


/*
 * Basically the init-er of cRosCom
 */
int cRosCom::Create( ros::NodeHandle *rosNode, cCanbus *canbus)
{
	// Get parameters
	ros::NodeHandle n("~");
	n.param<bool>(       "SwitchLeftRight",       SwitchLeftRight, false);
	n.param<bool>( "SwitchForwardBackward", SwitchForwardBackward, false);

	// Set canbus ref
	_canbus  = canbus;
	_rosNode = rosNode;

	// Subscribe to /canbus/speed
	_subSpeed = _rosNode->subscribe<flarb_msgs::DualMotorSpeedPtr>( "/canbus/speed", 1, &cRosCom::SendSpeed, this);

	// We publish at /canbus/encoder
	_pubEncoder = _rosNode->advertise<flarb_msgs::DualMotorEncoder>( "/canbus/encoder", 1);

	// return success
	return 0;
}


/*
 * Stuff is quite boring here, ain't it?
 */
int cRosCom::Destroy()
{
	return 0;
}


/**
 * We received a message on the canbus, publish if we got the right subscriber
 */
void cRosCom::MessageReceived( const struct CanMessage &canmessage)
{
	// Get opcode
	uint8_t opcode = canmessage.data[0];

	// crisis canbus protocol standard opcodes
	switch( opcode)
	{
		case crisis_header::OP_CRISIS_INVALID_FRAME:
			cout << "[OPCODE] ERROR: Invalid frame received from dev id " << canmessage.identifier << endl;
			break;

		case crisis_header::OP_HELLO:
			ProcessHelloMessage( canmessage);
			break;

		// An unknown opcode can still mean we have a device specific opcode
		default:

			// SpeedController device
			if( canmessage.identifier == (unsigned int)_devSpeedID)
			{
				ProcessDeviceSpeedMessage( canmessage);
				break;
			}

			// We got nothing
			cout << "[OPCODE] ERROR: Unknown opcode. canID" << canmessage.identifier;
			cout << ", Opcode " << opcode << endl;
			break;
	}
}


/**
 * We received a hello message, check the device type
 */
void cRosCom::ProcessHelloMessage( const struct CanMessage &canmessage)
{
	uint8_t device_type = canmessage.data[1];
	/* bytes from 2 to 5 contain the id */

	switch( device_type)
	{
		case crisis_hello::DUAL_DC_MOTOR_DRIVER:
			cout << "[OPCODE] Dual DC motor driver registered" << endl;
			_devSpeedID = canmessage.identifier;
			break;

		default:
			cout << "[OPCODE] ERROR: Unknown device. canID" << canmessage.identifier;
			cout << ", DevID " << device_type << endl;
			
			break;
	}
}


/**
 * We received a dev Speed specific message
 */
void cRosCom::ProcessDeviceSpeedMessage( const struct CanMessage &canmessage)
{
	uint8_t opcode = canmessage.data[0];

	switch( opcode)
	{
		// We received a motor status message
		case dual_motor_driver_opcodes::OP_STATUS:
		{
			flarb_msgs::DualMotorEncoder msg;
			mix_t val;

			val.c[0] = canmessage.data[1];
			val.c[1] = canmessage.data[2];
			val.c[2] = canmessage.data[3];
			val.c[3] = canmessage.data[4];
			// TODO get 5th byte, only when Leon updates firmware

			msg.speed_left  = val.s16.hi;
			msg.speed_right = val.s16.lo;

			// Enter data depending on we need to switch left/right
			if( SwitchLeftRight)
			{
				msg.speed_right = val.s16.hi;
				msg.speed_left  = val.s16.lo;
			}
			else
			{
				msg.speed_left  = val.s16.hi;
				msg.speed_right = val.s16.lo;
			}

			_pubEncoder.publish( msg);

			break;
		}

		default:
			cout << "[OPCODE] ERROR: Unknown opcode. canID" << canmessage.identifier;
			cout << ", Opcode " << opcode << endl;

			break;
	}
}


/**
 * Puts an speed message on the Canbus
 */
void cRosCom::SendSpeed( const flarb_msgs::DualMotorSpeedPtr msg)
{
	if( _devSpeedID == -1)
	{
		cout << "[ROSCOM] DUAL_DC_MOTOR_DRIVER not connected!" << endl;
		return;
	}

	// Construct a canbus message
	struct CanMessage canmessage;
	canmessage.identifier = _devSpeedID;
	canmessage.length     = 6;

	// Val
	mix_t val;

	// Enter data depending on we need to switch left/right
	if( SwitchLeftRight)
	{
		val.s16.hi = msg->speed_right;
		val.s16.lo = msg->speed_left;
	}
	else
	{
		val.s16.hi = msg->speed_left;
		val.s16.lo = msg->speed_right;
	}

	// Are forward and backward switched?
	if( SwitchForwardBackward)
	{
		val.s16.hi *= -1;
		val.s16.lo *= -1;
	}

	// Copy data
	canmessage.data[0] = dual_motor_driver_opcodes::OP_SET_SPEED;
	canmessage.data[1] = val.c[0];
	canmessage.data[2] = val.c[1];
	canmessage.data[3] = val.c[2];
	canmessage.data[4] = val.c[3];
	canmessage.data[5] = msg->flags;

	// Forward message to canbus
	_canbus->PortSend( canmessage);
}

