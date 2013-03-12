#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"

/*
* Main controller class of the rgb camera
*/
class cController
{
public:
    // Constructor
    cController() : _count( 0) {}

    // Functions executed at the beginning and end of the Application
    bool Create();
    void Destroy();

    // Updates the Server
    void Update();

private:
	
	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// We are publishing shizzle
	ros::Publisher  _rosTopic;

	int _count;
	
};

#endif // CLASS_CONTROLLER_H
