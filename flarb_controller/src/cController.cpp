// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_VDMixer/State.h"
#include "flarb_controller/cController.h"
#include "flarb_controller/WaypointVector.h"
#include "flarb_controller/controllers/cSegmentStart.h"
#include "flarb_controller/controllers/cSegmentFollow.h"
#include "flarb_controller/controllers/cSegmentFind.h"
#include "flarb_controller/controllers/cAvoidObstacle.h"
#include "flarb_controller/controllers/cPlantQuality.h"
#include "flarb_controller/controllers/cFreeRun.h"
using namespace std;


/*
 *Functions executed at the beginning and end of the Node
 */
bool cController::Create()
{
	// Init RosCom object
	_rosCom.Create( &_rosNode, this);

	// Create sub-controllers
	_segmentStart  = new cSegmentStart();
	_segmentFollow = new cSegmentFollow();
	_segmentFind   = new cSegmentFind();
	_avoidObstacle = new cAvoidObstacle();
	_plantQuality  = new cPlantQuality();
	_freeRun       = new cFreeRun();
	
	// Init sub-controllers
	_segmentStart->Create();
	_segmentFollow->Create();
	_segmentFind->Create();
	_avoidObstacle->Create();
	_plantQuality->Create();
	_freeRun->Create();

	return true;
}


/*
 * Executed when the Node is exiting
 */
void cController::Destroy()
{
	// De-init sub-controllers
	_segmentStart->Destroy();
	_segmentFollow->Destroy();
	_segmentFind->Destroy();
	_avoidObstacle->Destroy();
	_plantQuality->Destroy();
	_freeRun->Destroy();

	// Delete the subs
	delete _segmentStart;
	delete _segmentFollow;
	delete _segmentFind;
	delete _avoidObstacle;
	delete _plantQuality;
	delete _freeRun;
}


/*
 * Updates the controller obj
 * Here we can add extra code, which gets executed every X seconds (defined in main.cpp)
 */
void cController::Update()
{
	
}

/*
 * Gets called when we receive something from the smartphone
 * Example input: S - 3L - 0 - 2L - 2R - 1R - 5L - F
 */
void cController::SmartphoneCallback( const std::string &str)
{
	// We received a correct string, create a new cInputString obj
	if( str[0] == 'S')
	{
		_inputString = cInputString( str);
		_state = STATE_FIND_SEGMENT;
	}
	// We start in freerun mode
	else if( str[0] == 'F')
	{
		_state = STATE_FREERUN;
	}
	// Invalid format
	else
	{
		cout << "Smartphone input invalid!" << endl;
	}
}

/*
 * Gets called by cRosCom when we received a /map message
 */
void cController::MapCallback( cMap &map)
{
	// Vector
	tVector vector;
	// TODO grab VDMixer State


	// Always execute AvoidObstacle sub-controller
	/*bool ret = _avoidObstacle.Execute( vector, map);

	// Did AvoidObstacle return true, and we ain't in AvoidObstacle mode? Save state
	if( ret == true && _state != STATE_AVOID_OBSTACLE)
	{
		_statePrevious = _state;
		_state = STATE_AVOID_OBSTACLE;
	}
	// If we are in state AvoidObstacle but it failed, reset State to previous
	else if( ret == false && _state == STATE_AVOID_OBSTACLE)
		_state = _statePrevious;


	// If AvoidObstacle hasn't found anything, try Find/Follow-segment twice
	int turns = 2;
	for(; ret == false && turns > 0; turns--)
	{
		switch( _state)
		{
			case STATE_FOLLOW_SEGMENT:
				ret = _followSegment.Execute( vector, map);
				if( ret == false)
					_state = STATE_FIND_SEGMENT;
				break;

			case STATE_FIND_SEGMENT:
				ret = _findSegment.Execute( vector, map);
				if( ret == false)
					_state = STATE_FOLLOW_SEGMENT;
				break;

			default:
				cout << "cController: This should never happen!!" << endl;
				ret = true;
				break;
		}
	}

	// Find/Follow-segment controllers couldn't come up with a solution. We are stuck now!
	if( turns == 0 && ret == false)
	{
		cout << "cController: We are bloody stuck :(" << endl;

		// Failsafe, set the values to 0
		vector = tVector( 0, 0);
	}


	// We have a filled message by now, publish it
	// msg struct, which we'll be sending
	flarb_controller::WaypointVector msg;
	msg.x = vector.getX();
	msg.y = vector.getY();
	_rosCom.PublishWaypoint( msg);*/
}

