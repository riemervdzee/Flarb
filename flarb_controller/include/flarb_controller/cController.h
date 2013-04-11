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
    
    // Functions executed at the beginning and end of the Application
    bool Create();
    void Destroy();

    // Updates the Node
    void Update();

private:
	
	// Reference to the ros node handle
	ros::NodeHandle _rosNode;


	
};

#endif // CLASS_CONTROLLER_H
