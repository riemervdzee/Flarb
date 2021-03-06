#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"
#include "flarb_compass/cSerial.h"


// Class define
#define BUFFER_SIZE 512


/*
* Main controller class of the example node
*/
class cController
{
public:
	// C-tor
	cController() : UseCalibrated( true) {}

    // Functions executed at the beginning and end of the Application
    bool Create();
    void Destroy();

    // Updates the Node
    void Update();
    
    // Functions called when opening
	int Openport();
	int configure();

	// Calibrate
	int Calibration();

private:
	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// We are publishing
	ros::Publisher _Compass;

	// Serial obj
	cSerial _serial;

	// Actual reads data from the port
	int readDeviceCal();
	int readDeviceRaw();

	// Only used in raw mode, to clear unwanted data
	char Buffer[ BUFFER_SIZE];

	// Whether to use calibrated or raw
	bool UseCalibrated;
};

#endif // CLASS_CONTROLLER_H
