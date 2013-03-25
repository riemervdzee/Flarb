// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>
#include <termios.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_canbus/cController.h"
using namespace std;

#define SERIAL_PORT  "/dev/ttyUSB0"
#define SERIAL_BAUD  B9600


// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	// Topic name / buffer
	//_rosTopic = _rosNode.advertise<std_msgs::String>( "images", 100);
	
	//
	_canbus.PortOpen( SERIAL_PORT, SERIAL_BAUD, 6);

	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	_canbus.PortClose();
}

//static int _count = 0;

// Updates the controller obj
void cController::Update()
{
	// Assemble message
	/*std_msgs::String msg;
	std::stringstream ss;
	ss << "hello world " << _count;
	msg.data = ss.str();

	// Publish
	_rosTopic.publish( msg);*/
	
	//_canbus.SendCommand( "N\r", 2);

	// Check for canbus errors
	_canbus.CheckErrors();

	// Check for messages
	_canbus.PortRead( NULL);

	if( _count < 4)
	{
		_count++;
		CanMessage msg;
		msg.identifier = 2;
		msg.length     = 2;
		msg.data[0]    = '8';
		msg.data[1]    = '0';
	
		_canbus.PortSend( &msg);
	}
}

