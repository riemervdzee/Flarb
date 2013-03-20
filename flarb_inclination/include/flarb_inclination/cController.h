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
    //cController() : _count( 0) {}
	cController() : tries( 0) {}
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
	ros::Publisher _data;
	//Opening FileDescriptor
	int Openport();
	//File descriptor for the port 
	int fd;
	
	int getChar(); 
	int tries;
	
	//Temporary strorage for export data	
	float xaxis;
	float yaxis;
	
};

#endif // CLASS_CONTROLLER_H
