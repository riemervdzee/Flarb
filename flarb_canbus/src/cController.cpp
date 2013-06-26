// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_canbus/cController.h"
using namespace std;


// Canbus settings
#define SERIAL_PORT   "/dev/ttyUSB0"   // Device filename used in linux
#define SERIAL_BAUD   9600             // baudrate used, Lawicel has auto-baud
#define CANBUS_SPEED  4                // Canbus speed, see the Lawicel docs

// Tells whether we should reopen the canbus on error
#define RESET_ON_ERRORS 1

/*
 * Functions executed at the beginning and end of the Node
 */
bool cController::Create()
{
	// Init RosCom object first, then the canbus
	_roscom.Create( &_rosNode, &_canbus);

	// Init Canbus object
	int ret = -1;
	while (ret != 0 && ros::ok())
	{
		ret = _canbus.PortOpen( SERIAL_PORT, SERIAL_BAUD, CANBUS_SPEED, &_roscom, true);
		sleep( 1);
	}

	return true;
}

/*
 * Executed when the Node is exiting
 */
void cController::Destroy()
{
	_canbus.PortClose();
}

/*
 * Updates the controller obj
 */
void cController::Update()
{
	// Check for canbus errors
#if RESET_ON_ERRORS
	if(_canbus.CheckErrors())
	{
		_canbus.PortClose();
		_canbus.PortOpen( SERIAL_PORT, SERIAL_BAUD, CANBUS_SPEED, &_roscom, false);
	}
#else
	_canbus.CheckErrors();
#endif

	// Check for messages
	_canbus.PortRead();
}

