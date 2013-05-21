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
#define CANBUS_SPEED  4                // Canbus speed, see the Lawicel docs


/*
 * Functions executed at the beginning and end of the Node
 */
bool cController::Create()
{
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
	// Check for canbus errors
	_canbus.CheckErrors();

	// Check for messages
	_canbus.PortRead();
}

