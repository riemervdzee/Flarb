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
void cController::CallbackMap( const cImage &image)
{
	// Delegate which sub-controller should get executed
	flarb_controller::WaypointVector msg;

	// Always execute AvoidObstacle sub-controller
	bool ret = _avoidObstacle.Execute( msg, image);

	// Are we currently in avoid_obstacle mode?
	if( _state == STATE_AVOID_OBSTACLE)
	{
		// AvoidObstacle is succesful!
		if( !ret)
			_state = _statePrevious;
	}
	// We currently ain't in STATE_AVOID_OBSTACLE, but it returned true
	// Save current state, set current to AvoidObstacle
	else if( ret)
	{
		_statePrevious = _state;
		_state = STATE_AVOID_OBSTACLE;
	}

	// AvoidObstacle failed
	if( !ret)
	{
		// TODO fix logic way better than this..
		while( ret == false)
		{
			//TODO if both controllers fail, we end up in an endless loop..
			switch( _state)
			{
				case STATE_FOLLOW_SEGMENT:
					ret = _followSegment.Execute( msg, image);
					if( ret == false)
						_state = STATE_FIND_SEGMENT;
					break;

				case STATE_FIND_SEGMENT:
					ret = _findSegment.Execute( msg, image);
					if( ret == false)
						_state = STATE_FOLLOW_SEGMENT;
					break;

				default:
					cout << "cController: This should never happen!! arg" << endl;
					ret = true;
					break;
			};
		}
	}
	
	// We have a filled message by now
	_rosCom.PublishWaypoint( msg);
}

