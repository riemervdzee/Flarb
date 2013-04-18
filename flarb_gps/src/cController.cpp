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

// Settings
#define DEV_PORT		"/dev/ttyUSB0" 	//port 
#define BAUD_RATE		B4800			//Baudrate port

using namespace std;

/*
 *	Functions executed at the beginning and end of the Node
 */
int cController::Create()
{
	// Topic name / buffer
	_GGA = _rosNode.advertise<flarb_gps::GGA>("NMEA/GGA", 1);
	_RMC = _rosNode.advertise<flarb_gps::RMC>("NMEA/RMC", 1);
	Openport();
	readDevice(0);
	return 0;
}

/* 
 *	Executed when the Node is exiting
 */
void cController::Destroy()
{
	//always close the port
	_serial.PortClose();
	ros::shutdown();
}

/*
 *	Updates the controller obj
 */
void cController::Update()
{
	readDevice(1);
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
	char buffer[255];
	char buffRaw[255];
	memset (buffRaw,'-',255);
	int a = _serial.Read(buffRaw, sizeof(buffRaw));
	if(a > 0){
		if(start){
			//fill with '-'	
			memset (buffer,'-',255);
			//last time how much 
			char * savePtr = strchr(buff, '-');
			//copy backupped buffer
			char * s = savePtr;			
			s--;
			//copy buff in working buffer
			memcpy (buffer, buff, *s);
			//length of our new buffer
			char * t = strchr(buffRaw,'-');
			//copy raw buffer afther working buffer
			memcpy (buffer + *savePtr, buffRaw, *t );
		}
		else{
			*buffer = *buffRaw;
		}
		
		//*buffer = *buffRaw;
		
		char * pch;
		char * pch_cr;
		//char * pch_lf;
	  	pch = strchr(buffer,'$');	
		if(pch != NULL){
			while(pch != NULL){	
				pch_cr = strchr(pch + 1,'\r');
				if(pch_cr != NULL){
					char NMEA[pch_cr - pch];
					memcpy( NMEA, pch, (pch_cr - pch));
					//Checksum
					cout<< NMEA <<endl;
					if(checksum(buffer) > 0){
						cout<<"checksum succes"<<endl;
						getPackage(NMEA);
					}
				}
				pch = strchr(pch + 1,'$');		
			}
			memset (buff, '-', 255);
			memcpy(buff, pch, (strchr(buffer, '-') - pch));
			cout<< buff << endl;
			cout<<"white line" << endl;
		}	
	
	}
	return 1;
}

/*
 *	Get Data from package
 */
int cController::getPackage(char* dataPack)
{
		
	//Check GGA Message
	if(dataPack[3] == 'G' && dataPack[4] == 'G' && dataPack[5] == 'A'){
		//gga message
		flarb_gps::GGA gga;	

		char * pch;
		int counter = 1;		
		
		pch=strchr(dataPack , ',');
		while (pch != NULL)
		{
			GGAMessage( pch + 1, counter, gga );
			pch = strchr( pch + 1 , ',');
			counter++;
		}
		//_NMEA.publish(gga);
		cout<<"publish gga"<<endl;
		return 1;
	}
	//check rmc message
	else if(dataPack[3] == 'R' && dataPack[4] == 'M' && dataPack[5] == 'C'){
		//vtg message
		flarb_gps::RMC rmc;
		
		char * pch;
		int counter = 1;		
		
		pch=strchr(dataPack , ',');
		while (pch != NULL)
		{
			RMCMessage(pch + 1, counter, rmc);
			pch = strchr(pch + 1 , ',');
			counter++;
		}
		//_NMEA.publish(rmc);
		cout<<"publish RMC"<<endl;
		return 1;
	}
	else if(dataPack[3] == 'V' && dataPack[4] == 'T' && dataPack[5] == 'G'){}
	else{
		return 0 ;
	}
	return 1;
}

/*
 *	Fill VTG message
 */
int cController::RMCMessage(char *data, int counter, flarb_gps::RMC rmc){
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
int cController::GGAMessage(char *data, int counter, flarb_gps::GGA gga){
	
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


