#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"
#include "flarb_inclination/cSerial.h"

/*
* Main controller class of the example node
*/
class cController
{
public:
    // Constructor
    cController() : _count( 0) {}

    // Functions executed at the beginning and end of the Application
    int Create();
    void Destroy();
	
    // Updates the Node
    void Update();

private:
	// Serial object
	cSerial _serial;

	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// We are publishing shizzle
	ros::Publisher  _rosTopic;
	//Opening FileDescriptor
	int open_port();
	//File descriptor for the port 
	int fd;
	//Getting package
	int getChar(); 
	int _count;
	float xaxis;
	float yaxis;
	char datapackage[];
};

#endif // CLASS_CONTROLLER_H
