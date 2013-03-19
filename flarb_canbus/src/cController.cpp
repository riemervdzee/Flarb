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
	cout << "canbus result " << _canbus.PortOpen( SERIAL_PORT, SERIAL_BAUD, 6) << endl;

	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	_canbus.PortClose();
}

// Updates the controller obj
void cController::Update()
{
	// Increase count
	/*_count++;

	// Assemble message
	std_msgs::String msg;
	std::stringstream ss;
	ss << "hello world " << _count;
	msg.data = ss.str();

	// Publish
	_rosTopic.publish( msg);*/
	
	//_canbus.SendCommand( "N\r", 2);
	
	_canbus.PortRead( NULL);
}

