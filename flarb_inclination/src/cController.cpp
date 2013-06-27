// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "flarb_msgs/Axis.h"
#include "flarb_inclination/cController.h"
#include "flarb_inclination/cSerial.h"

#include <stdio.h>   					// Standard input/output definitions
#include <string.h>  					// String function definitions 
#include <unistd.h>  					// UNIX standard function definitions 
#include <fcntl.h>   					// File control definitions 
#include <errno.h>   					//	Error number definitions 
#include <termios.h> 					//	POSIX terminal control definitions 

// Settings
#define DEV_PORT		"/dev/ttyS1" 	//port 
#define BAUD_RATE		B9600			//Baudrate port

using namespace std;

/*
 *	Functions executed at the beginning and end of the Node
 */
int cController::Create()
{
	// Topic name / buffer
	_data = _rosNode.advertise<flarb_msgs::Axis>("sensor/inclination", 100);

	//open ttys1
	Openport();

	return 0;
}

/* 
 *	Executed when the Node is exiting
 */
void cController::Destroy()
{
	//always close the port
	_serial.PortClose();
	//now shutdown ros
	ros::shutdown();
}

/*
 *	Updates the controller obj
 */
void cController::Update()
{
	//if axis updated
	if( getPackage() == 1){
		//create message
		flarb_msgs::Axis msg;
		//fill message
		msg.x = xaxis;
		msg.y = yaxis;
		//Publish
		_data.publish(msg);
	}
}

/*
 * 'open_port()' - Open serial port.
 *
 * failed == -1, succes == 0;
 *
 * 9600 Baudrate, ASCII, 8 data bits, 1 stop bit, no parity
 */
int cController::Openport()
{
	int ret = -1;

	while( ret != 0 && ros::ok())
	{
		ret = _serial.PortOpen(DEV_PORT, BAUD_RATE);

		if( ret != 0)
			sleep(1);
	}
	if(ret == 0)
		cout<< "Opened serial " <<endl;
	

	return 0;
}


/*
 * This method will check if there is an packagage available.
 * if so it will return two floats X and Y 
 * 
 * layout of the package that's send over the RS232 Port
 * 22 Byte 
 * Layout: < D0 ... D21>
 * 		D0 ... D10	= “X=±xx.xxx“, <CR>, <LF> with D2 = sign (+ or -), D5 = decimal point
 * 		D11 ... D21 = “Y=±xx.xxx“, <CR>, <LF> with D13= sign (+ or -), D16 = decimal point
 *		Example: "X=+01.123\r\nY=-12.123\r\n"
 *		Return: 0 = Serial ok, -1 = Serial bad 
 */
int cController::getPackage()
{
	//bool XX = false;
	//bool YY = false;

	char r;
	while(r != 'X') {
		_serial.Read(&r, 1);
	}

	char buffer[64];
	int x;
	_serial.Read(&r, 1); // =
	for(x = 0; buffer[x] != '\r' && x < sizeof(buffer); x++) {
		_serial.Read(&buffer[x], 1);
	}
	if(x == sizeof(buffer)) {
		return -1;
	}
	buffer[x] = '\0';
	xaxis = atof(buffer);

	while(r != 'Y') {
		_serial.Read(&r, 1);
	}

	_serial.Read(&r, 1); // =
	for(x = 0; buffer[x] != '\r' && x < sizeof(buffer); x++) {
		_serial.Read(&buffer[x], 1);
	}
	if(x == sizeof(buffer)) {
		return -1;
	}
	buffer[x] = '\0';
	yaxis = atof(buffer);
	return 1;
}
