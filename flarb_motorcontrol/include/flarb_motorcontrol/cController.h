#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"

#include "flarb_motorcontrol/cRosCom.h"

// The width of the chassis in meters
#define CHASSIS_WIDTH 0.4f

/*
 * Main controller class of the example node
 */
class cController
{
public:
	// C-tor for default vals
	cController() : _inputLeft(0.0f), _inputRight(0.0f) {}

	// Functions executed at the begining and end of the Application
	bool Create();
	void Destroy();

	// Calculates motor strengths based on input
	void SetWaypoint( float x, float y);

	// Try to obtain the output values for the motor via PID and input vals
	void Update();

private:
	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// RosCom obj, deals with the communication towards other ROS nodes
	cRosCom _roscom;

	// Motor strengths as requested by /steering/waypoint/
	float _inputLeft, _inputRight;
};

#endif // CLASS_CONTROLLER_H
