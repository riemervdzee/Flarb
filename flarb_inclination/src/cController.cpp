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
	char buffer[255];  /* Input buffer 				*/
	bool XX = false;
	bool YY = false;


	int a = _serial.Read(buffer, sizeof(buffer));
	if(a != 0){
		for(int i = 0; i < a; i++){
			if(buffer[i] == 'X' && ( buffer[i + 9] == '\r'))
			{
				int cntx = 0;
		        char x[5];
		        for (int j = 2+i; j < 9+i; j++){
		                x[cntx]=buffer[j];
		                cntx++;
		        }
		        xaxis = ::atof(x);
				if(x[4] == '+')
					xaxis = +25;
				if(x[4] == '-')
					xaxis = -25;
				XX =true;
				//cout<<"Found X: "<< xaxis<< endl;
			
			}
			if(buffer[i] == 'Y' && ( buffer[i + 9] == '\r'))
			{
				int cnty = 0;
		        char y[5];
		        for (int j = 2+i; j < 9+i; j++){
		                y[cnty]=buffer[j];
		                cnty++;
		        }
		        yaxis = ::atof(y);
				if(y[4] == '+') 
					yaxis = +25;
				if(y[4] == '-')
					yaxis = -25;
				YY=true;
				//cout<<"Found Y: "<< yaxis<< endl;
			}
			if(XX && YY){
				return (1);
			}
		}
	}
	return (-1);			
}


