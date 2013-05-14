// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_controller/cController.h"
#include "flarb_controller/WaypointVector.h"
using namespace std;

/*
 * Constructor / C-tor
 */
// TODO we should start in STATE_INIT
cController::cController() : _state( STATE_FOLLOW_SEGMENT) {}


/*
 *Functions executed at the beginning and end of the Node
 */
bool cController::Create()
{
	// Init RosCom object
	_rosCom.Create( &_rosNode, this);
	
	// Init sub-controllers
	_followSegment.Create();
	_findSegment.Create();
	_avoidObstacle.Create();

	return true;
}


/*
 * Executed when the Node is exiting
 */
void cController::Destroy()
{
	// Deinit RosCom and subcontrollers
	_rosCom.Destroy();
	_followSegment.Destroy();
	_findSegment.Destroy();
	_avoidObstacle.Destroy();
}


/*
 * Updates the controller obj
 * Here we can add extra code, which gets executed every X seconds (defined in main.cpp)
 */
void cController::Update()
{
	
}


/*
 * Gets called by cRosCom when we received a /map message
 */
void cController::CallbackMap( cMap &map)
{
	// Vector
	tVector vector;


	// Always execute AvoidObstacle sub-controller
	bool ret = _avoidObstacle.Execute( vector, map);

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
	_rosCom.PublishWaypoint( msg);
}

