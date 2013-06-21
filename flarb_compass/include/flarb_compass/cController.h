#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"
#include "flarb_compass/cSerial.h"

// Program options
#define USE_CALIBRATED 1

// Class define
#define BUFFER_SIZE 512


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

	// We are publishing
	ros::Publisher _Compass;

	// Serial obj
	cSerial _serial;

	// Functions called when opening
	int Openport();
	int configure();

	// Actual reads data from the port
	int readDevice();

	// Only used in raw mode, to clear unwanted data
#if !USE_CALIBRATED
	char Buffer[ BUFFER_SIZE];
#endif

	//int Calibration();
};

#endif // CLASS_CONTROLLER_H
