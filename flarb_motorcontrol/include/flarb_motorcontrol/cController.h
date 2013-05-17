#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"

#include "flarb_motorcontrol/cRosCom.h"


/*
 * Main controller class of the example node
 */
class cController
{
public:
	// Functions executed at the begining and end of the Application
	bool Create();
	void Destroy();
	void Update();

private:
	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// RosCom obj, deals with the communication towards other ROS nodes
	cRosCom _roscom;
};

#endif // CLASS_CONTROLLER_H
