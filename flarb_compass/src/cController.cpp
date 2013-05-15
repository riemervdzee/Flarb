// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_compass/cController.h"
#include "flarb_compass/cSerial.h"
#include "flarb_compass/Compass.h"

#include <stdio.h>   					// Standard input/output definitions
#include <string.h>  					// String function definitions 
#include <unistd.h>  					// UNIX standard function definitions 
#include <fcntl.h>   					// File control definitions 
#include <errno.h>   					// Error number definitions 
#include <termios.h> 					// POSIX terminal control definitions 

// Settings
#define DEV_PORT		"/dev/ttyUSB0" 	//port 
#define BAUD_RATE		B9600			//Baudrate port

using namespace std;

// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	ptr = 0;
	Openport();	
	_Compass = _rosNode.advertise<flarb_compass::Compass>("sensor/compass", 1);
	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	_serial.Write("h", 1);
	usleep(100);
	_serial.Write("\n", 1);
	usleep(100);
	printf("shutdown");
	_serial.PortClose();	
	//now shutdown ros
	ros::shutdown();
	
}

// Updates the controller obj
void cController::Update()
{
	//cout<<"Update"<<endl;
	readDevice(0);
}

/*
 * 'open_port()' - Open serial port.
 *
 * failed == -1, succes == 0;
 *
 * 4800 Baudrate, ASCII, 8 data bits, 1 stop bit, no parity
 */
int cController::Openport()
{
	int ret = -1;

	//retry endless
	while( ret != 0 && ros::ok())
	{
		cout<<"Opening port" << endl;
		ret = _serial.PortOpen(DEV_PORT, BAUD_RATE);
		configure();
		sleep(1);		
		if( ret != 0)
			sleep(1);
	}
	//make contact
	if(ret == 0 && ros::ok()){
		cout<< "Port opened " <<endl;
		readDevice(1);
		return 0;	
	}
	//abort
	else{
		cout<< "Abort opening" << endl; 
		return -1; 
	}
}

/*
 * Configure compass for right data output
 */
int cController::configure()
{
	cout<<"Configuring"<<endl;
	//Configuring compass with delay
	usleep(10);
	_serial.Write("h", 1);
	usleep(10);
	_serial.Write("\n", 1);
	usleep(10);
	_serial.Write("go", 2);
	usleep(10);
	_serial.Write("\n", 1);
	usleep(10);
	return 0;
}

/* 
 *	For Reading device
 */	
int cController::readDevice(int start)
{
	char buffRaw[255];
	_serial.Read(buffRaw, sizeof(buffRaw));
	strncpy(Buffer, buffRaw, 255);
	getData(start);
	return 1;
}

/*
 *	get Data from stream
 */
int cController::getData(int start){
		char * pch = NULL;
		//char * pchNext = NULL;
		char * pchCheck = NULL;		
		pch = strpbrk(Buffer, "$");
		if(pch !=NULL)		
			//pchNext = strpbrk(pch + 1,"\n$");
			pchCheck = strpbrk(pch + 1, "*");
		if(pchCheck == NULL ){				
			return 0;
		}
		else{
			//cout<<"Start diff "<<endl;
			//int diff = pchNext - pch -1; 
			int size = (pchCheck - pch);
			
			char String[size];		
			int sCheck = strtol(pchCheck + 1,0,16);		
			strncpy(String, pch ,size);			
			
			//CheckSum
			int Checksumcalc = checksum(String);
			cout<<String<<endl;
			if (Checksumcalc == sCheck)
			{			
				//cout<< String<<endl;
				if( (strpbrk(String, "$") != NULL) && 
					(strpbrk(String, "C") != NULL) && 
					(strpbrk(String, "X") != NULL) && 
					(strpbrk(String, "Y") != NULL) && 
					(strpbrk(String, "T") != NULL))
				{
					
					//Compass heading
					char * cc = strpbrk(String, "C");
					char * xx = strpbrk(String, "X");
					char * yy = strpbrk(String, "Y");
					char * tt = strpbrk(String, "T");
					
					int sizeC = ( xx - cc );
					int sizeX = ( yy - xx );
					int sizeY = ( tt - yy );

					sizeC -= 1;
					sizeX -= 1;					
					sizeY -= 1;

					char angle[sizeC];
					char X[sizeX];
					char Y[sizeY];
					strncpy(angle, cc +1, sizeC);			
					strncpy(X, xx +1, sizeX);	
					strncpy(Y, yy +1, sizeY);

					flarb_compass::Compass msg;
					msg.north_angle = atof(angle);
					_Compass.publish(msg);
					cout<<"Published!"<< strtol(pch + 2,0,10)<<endl;
				}
				
			}
	}
	return 1;
}

/*
 *	XOR Checksum
 */
int cController::checksum(char *s) {
    int c = 0;
 
    while(*s)
        c ^= *s++;
 
    return c;
}
