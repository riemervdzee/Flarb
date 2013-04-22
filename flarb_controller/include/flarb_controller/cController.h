#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"

#include "flarb_controller/types/tVector.h"
#include "flarb_controller/types/tMatrix.h"

#include "flarb_controller/sInputString.h"
#include "flarb_controller/cImage.h"
#include "flarb_controller/cRosCom.h"
#include "flarb_controller/controllers/cFollowSegment.h"
#include "flarb_controller/controllers/cFindSegment.h"
#include "flarb_controller/controllers/cAvoidObstacle.h"


/*
 * Enumerates the possible states the segmentsolver can be in
 */
enum STATES {
	STATE_INIT,            // Init-state, we should check if everything is online
	STATE_STOPPED,         // We stopped doing anything
	STATE_FOLLOW_SEGMENT,  // We should execute: cFollowSegment
	STATE_FIND_SEGMENT,    // cFindSegment
	STATE_AVOID_OBSTACLE,  // cAvoidObstacle
};

/*
 * Main controller class of the example node
 */
class cController
{
public:
	// Constructor
	cController();

	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Updates the Node
	void Update();

	// Gets called by cRosCom when we received a /map message
	void CallbackMap( const cImage &image);

private:

	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// RosCom obj, deals with the communication towards other ROS nodes
	cRosCom _rosCom;

	// Our inputstring Manager
	sInputString _inputString;

	// Our current state
	enum STATES _state;

	//
	enum STATES _statePrevious;

	// Our sub-controllers
	cFollowSegment  _followSegment;
	cFindSegment    _findSegment;
	cAvoidObstacle  _avoidObstacle;
};

#endif // CLASS_CONTROLLER_H
