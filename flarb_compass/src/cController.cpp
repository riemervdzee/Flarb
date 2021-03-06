// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "flarb_msgs/Compass.h"
#include "flarb_msgs/Axis.h"

#include "flarb_compass/cController.h"
#include "flarb_compass/cSerial.h"

#include <stdio.h>   					// Standard input/output definitions
#include <string.h>  					// String function definitions 
#include <unistd.h>  					// UNIX standard function definitions 
#include <fcntl.h>   					// File control definitions 
#include <errno.h>   					// Error number definitions 
#include <termios.h> 					// POSIX terminal control definitions 
#include <math.h>


// Settings
#define DEV_PORT		"/dev/ttyS2"    //port
#define BAUD_RATE		9600            //Baudrate port


// Math extension
#define RAD2DEG  (180 / M_PI)
#define DEG2RAD  (M_PI / 180)

using namespace std;


// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	//
	ros::NodeHandle n("~");
	n.param<bool>( "UseCalibrated", UseCalibrated, true);
	cout << "UseCalibrated set to " << UseCalibrated << endl;

	// Open compass
	_Compass = _rosNode.advertise<flarb_msgs::Compass>("sensor/compass", 1);

	//subscribe topic
	Inclino = _rosNode.subscribe<flarb_msgs::Axis>("/sensor/inclination", 1, &cController::axismsgcb, this);
	Openport();

	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	_serial.writeString("h\n");

	cout << "shutdown" << endl;
	_serial.close();
	//now shutdown ros
	ros::shutdown();
}

// Updates the controller obj
void cController::Update()
{
	try {
		//cout<<"Update"<<endl;
		if( UseCalibrated)
			readDeviceCal();
		else
			readDeviceRaw();
	}
	catch(...){}
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

		try {
			// Open device
			if( !_serial.isOpen())
				_serial.open( DEV_PORT, BAUD_RATE);

			// Timeout for reads
			_serial.setTimeout( boost::posix_time::milliseconds(200));

			configure();

			// If all succeeded without throwing exceptions, set ret to 0 quitting the loop
			ret = 0;
		}
		catch(...){ sleep(1); cout<<"Reattempt opening" << endl;}
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
{;
	// Tell the compass to stop writing, if any
	_serial.writeString("h\n");

	// When using calibrated data, the compass sends a string X times
	if( UseCalibrated)
		_serial.writeString("go\n");

	return 0;
}


/* 
 *	For Reading device, the calibrated way
 */	
int cController::readDeviceCal()
{
	// Read till we got a response (ended with \n)
	string str = _serial.readStringUntil();

	// Process, format: $C269.64X-0.007394Y1.175806T24.8*1B
	char *chrC = strchr( &str[0], 'C');
	char *chrX = strchr( &str[0], 'X');
	char *chrY = strchr( &str[0], 'Y');

	// Check if the buffer is alrighty
	if( chrC != NULL && chrX != NULL && chrY != NULL)
	{
		float c = atof( chrC + 1);
		float x = atof( chrX + 1);
		float y = atof( chrY + 1);

		float z = sqrt(3*3 - x*x + y*y);
		if(isnan(z)) {
			z = 1.0f;
		}

		
		float pitch = static_cast<float>(axismsg.y)*DEG2RAD;
		float roll = static_cast<float>(axismsg.x)*DEG2RAD;

		flarb_msgs::Compass msg;
		msg.angle = c;
		
		msg.x = x*cos(pitch) + z*sin(pitch);
		msg.y = x*sin(roll)*sin(pitch)+ y * cos(roll) - z * sin(roll) * cos(pitch);

 		//msg.x     = cos((static_cast<float>(axismsg.x) * M_PI) / 180.0f ) * x;
		//msg.y     = cos((static_cast<float>(axismsg.y) * M_PI) / 180.0f ) * y;
		msg.angle = -(atan2(msg.y,msg.x)* RAD2DEG);

		cout << "Correction: " << axismsg.x << "/" << axismsg.y << "-->" << cos((static_cast<float>(axismsg.x) * M_PI)/180.0f) << " / " << cos((static_cast<float>(axismsg.y) * M_PI)/180.0f) << endl;
		if(msg.angle < 0)
			msg.angle += 360;

		cout<< "Original value x: " << x << "	Adjusted Value x: " << msg.x<< endl;
		cout<< "Original value y: " << y << "	Adjusted Value y: " << msg.y<< endl;
		cout << "Z axis: " << z << endl;
		cout<< "Original Angle: " << c << " 	new angle: " << msg.angle << endl; 				
		_Compass.publish(msg);

		//cout << str << endl;
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

/*
 * Callback inclination
 */
void cController::axismsgcb(const flarb_msgs::Axis msgr)
{
	axismsg = msgr;
}


/* 
 *	For Reading device, the raw output
 */	
int cController::readDeviceRaw()
{
	// Clear everything in the read buffer
	_serial.read( Buffer, sizeof(BUFFER_SIZE));
	Buffer[0] = 0;

	// Request a raw sample
	_serial.writeString( "sr?\n");

	// Read till we got a response (ended with \n)
	string str = _serial.readStringUntil();

	// Process, format: $raw,X-733Y35:E200*3C
	char *chrX = strchr( &str[0], 'X');
	char *chrY = strchr( &str[0], 'Y');

	// Check if the buffer is alrighty
	if( chrX != NULL && chrY != NULL)
	{
		float x = atoi( chrX + 1);
		float y = atoi( chrY + 1);

		flarb_msgs::Compass msg;
		msg.angle = -(atan2(y,x)* RAD2DEG);

		if( msg.angle < 0)
			msg.angle += 360;
		else if( msg.angle > 360)
			msg.angle -= 360;

		msg.x = x;
		msg.y = y;
		_Compass.publish(msg);

		//cout << str << endl;
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


#if 0

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
	int a = _serial.read(buffRaw, sizeof(buffRaw));
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
		_serial.writeString("h\n");

		_serial.writeString("factory\n");

		_serial.writeString("ps=6\n");

		_serial.writeString("em=e\n");

		_serial.writeString("et=e\n");

		_serial.writeString("ut=c\n");

		_serial.writeString("sn=t\n");

		_serial.writeString("mpcal=e\n");

		cout<<"Rotate the unit through two 360 degree circles while maintaining a level position"<<endl;
		cout<<"The rotations should be no faster than 30 seconds each."<<endl;
		cout<<"Press: 'Any key' when finished"<<endl;		
		_serial.writeString("go\n");

		// Wait till user input
		cin.ignore();
		cin.get();

		usleep(100);
		_serial.writeString("h\n");
		usleep(100);
		_serial.writeString("mpcal=d\n");
		usleep(100);
		_serial.writeString("save\n");
		usleep(100);
		_serial.writeString("go\n");
		cout<<"Config saved"<<endl;
		return 1;
	}
}


