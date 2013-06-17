#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"
#include "flarb_compass/Compass.h"
#include "flarb_compass/cSerial.h"

// Program options
#define USE_RAW 1

// Class define
#define BUFFER_SIZE 1024


/*
* Main controller class of the example node
*/
class cController
{
public:

    // Functions executed at the beginning and end of the Application
    bool Create();
    void Destroy();

    // Updates the Node
    void Update();

private:
	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// We are publishing shizzle
	ros::Publisher _Compass;
	flarb_compass::Compass msg;
	cSerial _serial;
	int Openport();
	int configure();
	int Calibration();

	int readDevice(int status);

#if (USE_RAW == 0)
	int getData(int status);
	int checksum(char *s);
	int ptr;
#endif
	char Buffer[ BUFFER_SIZE];
};

#endif // CLASS_CONTROLLER_H
