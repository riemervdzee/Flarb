#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include <cmath>
#include "ros/ros.h"
#include "flarb_motorcontrol/cRosCom.h"

// Defines
//TODO work out details
const float PULSES_PER_TURN = 50;               // TODO not known yet
const float METERS_PER_TURN = 0.10f * 2 * M_PI; // Circumference of wheel
const float VECTOR_MAX      = 0.5f;             // TODO decide
const float VECTOR_TIME     = 1;                // The amount of sec, the input needs to be executed

// Converts
const float PULSE2METER   = METERS_PER_TURN / PULSES_PER_TURN;
const float METER2PULSE   = PULSES_PER_TURN / METERS_PER_TURN;
const float VECTOR2SEC    = 1 / VECTOR_TIME;
const float VECTOR2MOTOR  = VECTOR2SEC * METER2PULSE;
const float VECTOR_MAXSEC = VECTOR2SEC * VECTOR_MAX;


/*
 * Main controller class of the example node
 */
class cController
{
public:
	// C-tor
	cController() : _framesToWait( 1), _framesWaiting( 0) {}

	// Functions executed at the begining and end of the Application
	bool Create( int hz);
	void Destroy();

	// Resets the framecount
	void WaypointReceived();

	// Try to obtain the output values for the motor via PID and input vals
	void Update();

private:
	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// We wait a few Update function calls before braking
	int _framesToWait;
	int _framesWaiting;

	// RosCom obj, deals with the communication towards other ROS nodes
	cRosCom _roscom;
};

#endif // CLASS_CONTROLLER_H
