// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "flarb_gps/GGA.h"
#include "flarb_gps/RMC.h"
#include "flarb_gps/cController.h"
#include "flarb_gps/cSerial.h"

#include <stdio.h>   					// Standard input/output definitions
#include <string.h>  					// String function definitions 
#include <unistd.h>  					// UNIX standard function definitions 
#include <fcntl.h>   					// File control definitions 
#include <errno.h>   					// Error number definitions 
#include <termios.h> 					// POSIX terminal control definitions 
#include <math.h>

// Settings
#define DEV_PORT		"/dev/ttyUSB0" 	//port 
#define BAUD_RATE		B4800			//Baudrate port

using namespace std;

/*
 *	Functions executed at the beginning and end of the Node
 */
int cController::Create()
{
	ptrb = 0;
	// Topic name / buffer
	_GGA = _rosNode.advertise<flarb_gps::GGA>("NMEA/GGA", 1);
	_RMC = _rosNode.advertise<flarb_gps::RMC>("NMEA/RMC", 1);
	Openport();
	
	return 0;
}

/* 
 *	Executed when the Node is exiting
 */
void cController::Destroy()
{
	_serial.PortClose();
	ros::shutdown();
}

/*
 *	Updates the controller obj
 */
void cController::Update()
{
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
		ret = _serial.PortOpen(DEV_PORT, BAUD_RATE);
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
		cout<< "Abort" << endl; 
		return -1; 
	}

}

/*
 *	Read data from Device
 */
int cController::readDevice(int start){
	//when it is getting out of hand
	if(ptrb > 512)
	{
		ptrb = 0;
	}
	
	char buffRaw[255];
	int a = _serial.Read(buffRaw, sizeof(buffRaw));
	if(a>0)
		memcpy(Buffer + ptrb, buffRaw, a);
	ptrb += a;
	int b = getData();
	return b;
}
	
/*
 * Getting data from stream
 */
int cController::getData(){
	int result=0;	
	char * pch = NULL;
	char * pchCheck = NULL;		
	pch = (char*) memchr(Buffer, '$', ptrb);
	char * pchNext = NULL;
	
	if(pch !=NULL)		
		pchCheck = (char*) memchr(pch + 1, '*', ptrb);
		if(pchCheck != NULL)
			pchNext = (char*) memchr(pchCheck + 1, '$', 6);
	if((pchCheck == NULL) || (pchNext == NULL) ){				
		return 0;
	}
	else
	{
		//get data stream
		int size = (pchCheck - pch);
		char String[size];
		strncpy(String, pch+1 ,size -1);
		String[size-1] = '\0';	

		//CheckSum
		char sCheck[3];
		strncpy(sCheck, pchCheck + 1,2);
		sCheck[2] = '\0';
		int checksumString = strtol(sCheck,0,16);		
		int Checksumcalc = checksum(String);
		
		if (Checksumcalc == checksumString)
		{	
			
			//GGA
			if (memcmp(String, "GPGGA", 5) == 0) 
			{
				result = 1;
			}				
			//RMC
			else if (memcmp(String, "GPRMC", 5) == 0) 
			{
				result = 2;
			}
			//VTG
			else if (memcmp(String, "GPVTG", 5) == 0) 
			{
				result = 3;
	
			}
			//RMB
			else if (memcmp(String, "GPRMB",5) == 0) 
			{
				result = 4;			
			
			}	
			//GSA
			else if (memcmp(String, "GPGSA",5) == 0) 
			{
				result = 5;	
						
			}
			//GSV
			else if (memcmp(String, "GPGSV",5) == 0) 
			{
				result = 6;	
								
			}
			else
			{	
				return -1;	
			}
			if(result < 3)
			{
				int counter = 1;		
				char *ptr_start=strchr(String , ',');
				char *ptr;
				while (ptr_start != NULL)
				{
					ptr = strchr(ptr_start + 1 , ',');
					if(ptr != NULL)
					{
						if((ptr - ptr_start) > 1)
						{
							int dataSize = ptr - ptr_start;
							char data[dataSize];
							data[dataSize -1] = '\0';
							memcpy(data, ptr_start +1, dataSize-1);
							//MessageType
							if(result == 1)	 {	GGAMessage( data, counter );	}
							if(result == 2)	 {	RMCMessage( data, counter );	}
							if(result == 3)	 {		}
							if(result == 4)	 {		}
							if(result == 5)	 {		}
							if(result == 6)	 {		}
						}
					}
					else
					{
						if((ptr_start + 1) != '\0')	{
							//last data
							if(result == 1)	 {	GGAMessage(ptr_start + 1, counter);	cout<<"GGA"<<endl;}
							if(result == 2)	 {	RMCMessage(ptr_start + 1, counter);	cout<<"RMC"<<endl;}
							if(result == 3)	 {		cout<<"VTG"<<endl;}
							if(result == 4)	 {		cout<<"RMB"<<endl;}
							if(result == 5)	 {		cout<<"GSA"<<endl;}
							if(result == 6)	 {		cout<<"GSV"<<endl;}
						}	
					}
					ptr_start = strchr( ptr_start + 1 , ',');
					counter++;
				}
				//publish
				if(result == 1) {	_GGA.publish(gga);	}
				if(result == 2) {	_RMC.publish(rmc);	}
				if(result == 3) {		}
				if(result == 4) {		}
				if(result == 5) {		}
				if(result == 6) {		}
			}
		
			
		}
		char Buftemp[1024];
		memcpy(Buftemp, pchNext, ptrb - (size + 4));
		memcpy(Buffer, Buftemp, 1024);
		ptrb -= size -1;
		getData();
	}
	return 0;
}


/*
 *	Fill VTG message
 */
int cController::RMCMessage(char *data, int counter){
	switch(counter){
		case 1:
			rmc.Track = atof (data);
			break;
		case 8:
			rmc.Speed = atof (data);
			break;
		}
	return 0;
}

/*
 *	Fill GGA Message
 */
int cController::GGAMessage(char *data, int counter){
	
	switch(counter){
		case 1:
			gga.Time = atof (data);
			break;
		case 2:
			gga.Latitude = atof (data);
			break;
		case 3:
			gga.NS = *data;
			break;
		case 4:
			gga.Longitude = atof (data);
			break;
		case 5:
			gga.EW = *data;
			break;
		case 6:
			gga.Quality = atoi (data);
			break;
		case 7:
			gga.Sattelites = atoi (data);
			break;
		case 8:
			gga.Dilution = atof(data);
			break;
		case 9:
			gga.Altitude = atof(data);
			break;
		case 11:
			gga.Separation = atof(data);
			break;
		case 13:
			gga.AgeBase = atof(data);
			break;
		case 14:
			gga.DiffBase = atof(data);
			break;
	}
	return 1;	
}			


/*
 * Checksum NMEA Data
 */
int cController::checksum(char *s) {
    int c = 0;
 
    while(*s)
        c ^= *s++;
 
    return c;
}


