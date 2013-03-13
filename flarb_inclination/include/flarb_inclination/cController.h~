#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"

/*
* Main controller class of the example node
*/
class cController
{
public:
    // Constructor
    cController() : _count( 0) {}

    // Functions executed at the beginning and end of the Application
    bool Create();
    void Destroy();
	
    // Updates the Node
    void Update();

private:
	
	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// We are publishing shizzle
	ros::Publisher  _rosTopic;
	//Opening FileDescriptor
	int open_port();
	//File descriptor for the port 
	int fd;
	//Getting package
	bool getChar(); 
	int _count;
	float xaxis;
	float yaxis;
	char datapackage[];
	 
	
};

#endif // CLASS_CONTROLLER_H
