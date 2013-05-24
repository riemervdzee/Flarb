#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"
#include "flarb_compass/Compass.h"
#include "flarb_compass/cSerial.h"

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
	int readDevice(int status);
	int getData(int status);
	int checksum(char *s);
	int configure();
	int Calibration();
	char Buffer[1024];
	int ptr;
};

#endif // CLASS_CONTROLLER_H
