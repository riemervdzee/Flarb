// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>
#include <termios.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_canbus/cController.h"
using namespace std;


#define SERIAL_PORT   "/dev/ttyUSB0"   // Device filename used in linux
#define SERIAL_BAUD   B9600            // baudrate used, Lawicel has auto-baud
#define CANBUS_SPEED  6                // Canbus speed, see the Lawicel docs


/*
 * C-tor, just set some values to 0
 */
cController::cController() : _hz( 0), _skipped( 0){}


/*
 * Functions executed at the beginning and end of the Node
 */
bool cController::Create( int hz)
{
	// Init _hz
	_hz = hz / CANBUS_HZ;
	_skipped = 0;

	// Init RosCom object first, then the canbus
	_roscom.Create( &_rosNode, &_canbus);

	// Init Canbus object
	_canbus.PortOpen( SERIAL_PORT, SERIAL_BAUD, CANBUS_SPEED, &_roscom);

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
	// See whether we need to check for errors
	_skipped++;
	if( _skipped >= _hz)
	{
		_skipped = 0;
		// Check for canbus errors
		_canbus.CheckErrors();
	}

	// Check for messages
	_canbus.PortRead();
}

