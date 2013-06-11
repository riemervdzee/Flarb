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
    cController() : x( 0), y( 0) {}

    // Functions executed at the beginning and end of the Application
    bool Create();
    void Destroy();

    // Updates the Node
    void Update();

private:

	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// Our subscribers and publishers
	ros::Publisher _pubVector; //  "/steering/waypoint/"

	// Our vector
	float x;
	float y;
};

#endif // CLASS_CONTROLLER_H
