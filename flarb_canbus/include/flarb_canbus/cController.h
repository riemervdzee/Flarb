#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"

#include "flarb_canbus/cCanbus.h"
#include "flarb_canbus/cRosCom.h"

// HZ we need to check the status of the canbus
#define CANBUS_HZ 3

/*
 * Main controller class of the example node
 */
class cController
{
public:
	// C-tor
	cController();

	// Functions executed at the beginning and end of the Application
	bool Create( int hz);
	void Destroy();

	// Updates the Node
	void Update();

private:
	// Controller_hz / canbus_hz = the amount we need to skip
	int _hz;

	// Current skipped controller_hz steps
	int _skipped;

	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// Canbus obj, deals with communication to the serial device
	cCanbus _canbus;

	// RosCom obj, deals with the communication towards other ROS nodes
	cRosCom _roscom;
};

#endif // CLASS_CONTROLLER_H
