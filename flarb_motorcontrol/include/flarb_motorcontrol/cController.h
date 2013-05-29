#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"

#include "flarb_motorcontrol/cRosCom.h"

// The width of the chassis in meters
//TODO work out details
#define CHASSIS_WIDTH 0.4f
#define MOTOR_MAX     100
#define VECTOR_MAX    0.5f
//TODO quite shitty
#define VEC2MOTOR     (MOTOR_MAX / VECTOR_MAX)
#define MOTOR2VEC     (VECTOR_MAX / MOTOR_MAX)

/*
 * Main controller class of the example node
 */
class cController
{
public:
	// Functions executed at the begining and end of the Application
	bool Create( int hz);
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
};

#endif // CLASS_CONTROLLER_H
