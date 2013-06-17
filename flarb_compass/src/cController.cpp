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
#include <math.h>


// Settings
#define DEV_PORT		"/dev/ttyUSB0" 	//port 
#define BAUD_RATE		B9600			//Baudrate port


// Math extension
#define RAD2DEG  (180 / M_PI)
#define DEG2RAD  (M_PI / 180)


using namespace std;

// Functions executed at the beginning and end of the Node
bool cController::Create()
{
#if (USE_RAW == 0)
	ptr = 0;
#endif
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
	cout << "shutdown" << endl;
	_serial.PortClose();	
	//now shutdown ros
	ros::shutdown();
	
}

// Updates the controller obj
void cController::Update()
{
	//cout<<"Update"<<endl;
	int res = readDevice(0);
	if(res == 0)	
		_Compass.publish(msg);
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
		
		//Calibration
#if 0
		cout<<"Do yo‎u want to calibrate? \nPress y for Yes"<<endl;
		char yes[] = "y";		
		char key[1];
	  	cin >> key;
		if((memcmp(yes, key,1) == 0))
		{	
			Calibration();
		}
#endif

		sleep(1);		
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

	// Clear read buffer
	//_serial.Read( Buffer, sizeof(BUFFER_SIZE));

	//Configuring compass with delay
	usleep(10);
	_serial.Write("h", 1);
	usleep(10);
	_serial.Write("\n", 1);
	usleep(10);

	// Clear read buffer
	_serial.Read( Buffer, sizeof(BUFFER_SIZE));

#if (USE_RAW == 0)
	_serial.Write("go", 2);
	usleep(10);
	_serial.Write("\n", 1);
	usleep(10);
#endif

	return 0;
}

#if USE_RAW

/* 
 *	For Reading device, Riemers way = raw
 */	
int cController::readDevice( int start)
{
	// Clear everything in the read buffer
	_serial.Read( Buffer, sizeof(BUFFER_SIZE));
	Buffer[0] = 0;

	// Request a raw sample
	_serial.Write("sr?\n", 4);
	/*usleep(10);
	_serial.Write("r", 1);
	usleep(10);
	_serial.Write("?", 1);
	usleep(10);
	_serial.Write("\r", 1);*/
	
	// Read response TODO wait for response?
	int len = 0;
	int attempts = 0;
	do {
		len += _serial.Read( Buffer + len, BUFFER_SIZE - len);
		//cout << len << endl;
		attempts++;
	}
	while ( ((strchr(Buffer, '$') == NULL) || strchr(Buffer, '\n') == NULL)  && 
		len < BUFFER_SIZE && attempts < 10 && usleep(1000) == 0);

	Buffer[len] = 0;

	// Process, format: $raw,X-733Y35:E200*3C
	char *chrX = strchr( Buffer, 'X');
	char *chrY = strchr( Buffer, 'Y');

	// Check if the buffer is alrighty
	if( chrX != NULL && chrY != NULL)
	{
		float x = atoi( chrX + 1);
		float y = atoi( chrY + 1);

		msg.north_angle = (atan2(y,x)* RAD2DEG);

		if( msg.north_angle < 0)
			msg.north_angle += 360;
		else if( msg.north_angle > 360)
			msg.north_angle -= 360;

		msg.x = x;
		msg.y = y;
		//msg.heading = heading;

		//cout << Buffer << endl;
		//cout << x << ", " << y << endl;
		//cout << msg.north_angle << endl;
	}
	else
	{
		static int i = 0;
		i++;
		cout << "thrown away " << i << endl;
	}

	return 0;
}

#else

/* 
 *	For Reading device, Daniels way (calibrated and all)
 */	
int cController::readDevice(int start)
{
	//getting out of hand
	if(ptr > 512)
	{
		ptr = 0;
	}
	char buffRaw[255];
	int a = _serial.Read(buffRaw, sizeof(buffRaw));
	strncpy(Buffer + ptr, buffRaw, a);
	ptr += a;
	int b = getData(start);
	return b;
}

/*
 *	get Data from stream
 */
int cController::getData(int start){
		char * pch = NULL;
		char * pchCheck = NULL;		
		pch = strpbrk(Buffer, "$");
		if(pch !=NULL)		
			pchCheck = strpbrk(pch + 1, "*");
		if(pchCheck == NULL ){				
			return -1;
		}
		else{
			int size = (pchCheck - pch) + 1;
			char String[size];
			int sCheck = strtol(pchCheck + 1,0,16);		
			strncpy(String, pch ,size -1);
			String[size-1] = '\0';	
			//CheckSum
			int Checksumcalc = checksum(String);
			if (Checksumcalc == sCheck)
			{	
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
					//termination string
					angle[sizeC-1] = '\0';
					X[sizeX-1] = '\0';
					Y[sizeY-1] = '\0';
					//cout<<X<<":"<<Y<<":"<<angle<<endl;
					float y = atof(Y);
					float x = atof(X);	
					float heading =0;					
					if(y > 0)
					{
						heading = 90 -(atan2(y,x)* RAD2DEG);
					}			
				    if(y<0)
					{ 
						heading = 270 -(atan2(y,x)* RAD2DEG);
					}
    				if(y==0 && x<0) 
						heading = 180.0;
    				if(y==0 && x>0) 
						heading = 0.0;

					msg.north_angle = atof(angle);
					msg.x = x;
					msg.y = y;
					msg.heading = heading;
					
					
					char * pchNext = NULL;
					char * stringLoc = String;
					pchNext = strpbrk(pch + 1, "$");
					if(pchNext != NULL){
						int diff = pchNext - pch -1;
						char Buftemp[1024];
						memcpy(Buftemp, Buffer, diff);
						memcpy(Buffer, Buftemp, 1024);
						ptr = 0;
						getData(1);
					}
					ptr = 0;
					return 0;					
				}
				//all other trash
				else
				{
					char * pchNext = NULL;
					char * stringLoc = String;
					pchNext = strpbrk(pch + 1, "$");
					if(pchNext != NULL){
						int diff = pchNext - pch -1;
						char Buftemp[1024];
						memcpy(Buftemp, Buffer, diff);
						memcpy(Buffer, Buftemp, 1024);
						ptr = 0;
					}
					return -1;
				}
			}
	}
	return 0;
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
#endif

/*
 *	Calibration Mode
 */
int cController::Calibration()
{
	{	
	  	cout<<"Starting Calibration"<<endl;
		sleep(3);
		_serial.Write("h", 1);
		usleep(100);
		_serial.Write("\n", 1);
		usleep(100);
		_serial.Write("factory", 7);
		usleep(100);
		_serial.Write("\n", 1);
		usleep(100);
		_serial.Write("ps=6", 4);
		usleep(100);
		_serial.Write("\n", 1);
		usleep(100);
		_serial.Write("em=e", 4);
		usleep(100);
		_serial.Write("\n", 1);
		usleep(100);
		_serial.Write("et=e", 4);
		usleep(100);
		_serial.Write("\n", 1);
		usleep(100);
		_serial.Write("ut=c", 4);
		usleep(100);
		_serial.Write("\n", 1);
		usleep(100);
		_serial.Write("sn=t", 4);
		usleep(100);
		_serial.Write("\n", 1);
		usleep(100);
		_serial.Write("mpcal=e", 7);
		usleep(100);
		_serial.Write("\n", 1);
		usleep(100);
		cout<<"Rotate the unit through two 360 degree circles while maintaining a level position"<<endl;
		cout<<"The rotations should be no faster than 30 seconds each."<<endl;
		cout<<"Press: 'Any key' when finished"<<endl;		
		_serial.Write("go", 2);
	    usleep(100);
		_serial.Write("\n", 1);
		usleep(100);

		// Wait till user input
		cin.ignore();
		cin.get();

		usleep(100);
		_serial.Write("h", 1);
		usleep(100);
		_serial.Write("mpcal=d", 7);
		usleep(100);
		_serial.Write("save", 4);
		usleep(100);
		_serial.Write("go", 2);
		cout<<"Config saved"<<endl;
		return 1;
	}
}
