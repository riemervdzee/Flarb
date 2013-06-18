#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include <string>
#include "ros/ros.h"

#include "flarb_controller/types/tVector.h"
#include "flarb_controller/types/tMatrix.h"
#include "flarb_controller/cInputString.h"
#include "flarb_controller/cRosCom.h"

// Forward declaration of our sub-controllers
class cSegmentStart;
class cSegmentFollow;
class cSegmentFind;
class cAvoidObstacle;
class cPlantQuality;
class cFreeRun;


/*
 * Enumerates the possible states the segmentsolver can be in
 */
enum STATES {
	STATE_INIT,            // Init-state, we should check if everything is online
	STATE_STOPPED,         // We stopped doing anything
	STATE_FREERUN,         // We should execute: cFreerun
	STATE_FOLLOW_SEGMENT,  // cFollowSegment
	STATE_FIND_SEGMENT,    // cFindSegment
	STATE_AVOID_OBSTACLE,  // cAvoidObstacle
};

/*
 * The return states of the subcontrollers
 */
enum SUBRETURN {
	RET_SUCCESS,
	RET_NEXT,
	RET_BLOCKED,
};


/*
 * Main controller class of the example node
 */
class cController
{
public:
	// Constructor
	cController() : _state( STATE_INIT), _blocked( false), _segmentStart( NULL), _segmentFollow( NULL),
		_segmentFind( NULL), _avoidObstacle( NULL), _plantQuality( NULL), _freeRun( NULL) {}

	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Updates the Node
	void Update();

	// Gets called by cRosCom when we received a /map message
	void MapCallback( cMap &map);

	// Gets called when we receive something from the smartphone
	void SmartphoneCallback( const std::string &str);

private:

	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// RosCom obj, deals with the communication towards other ROS nodes
	cRosCom _rosCom;

	// Our inputstring Manager
	cInputString _inputString;

	// Our current state
	enum STATES _state;

	// Are we in AvoidObstacle state-solver?
	bool _blocked;

	// Our sub-controllers
	cSegmentStart   *_segmentStart;
	cSegmentFollow  *_segmentFollow;
	cSegmentFind    *_segmentFind;
	cAvoidObstacle  *_avoidObstacle;
	cPlantQuality   *_plantQuality;
	cFreeRun        *_freeRun;
};

#endif // CLASS_CONTROLLER_H
