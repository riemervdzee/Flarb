// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_msgs/VDState.h"
#include "flarb_msgs/WaypointVector.h"
#include "flarb_controller/cController.h"
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

	// Params
	ros::NodeHandle n("~");
	double StartCheckRange;
	double StartSpeed;
	double FollowExtraRadius;
	double FollowSpeed;
	double FollowOffset;
	double FollowDecBlocked;
	double FindSpeed;
	double FindSpeedAngle;
	double FindGoalAngle;
	double FindSpeedFollow;
	double AvoidWaitTime;
	double AvoidSpeed;
	double AvoidGoalAngle;

	n.param<double>(   "StartCheckRange",   StartCheckRange, 0.50);
	n.param<double>(        "StartSpeed",        StartSpeed, 0.15);
	n.param<double>( "FollowExtraRadius", FollowExtraRadius, 0.02);
	n.param<double>(       "FollowSpeed",       FollowSpeed, 0.17);
	n.param<double>(      "FollowOffset",      FollowOffset, 12.0);
	n.param<double>(  "FollowDecBlocked",  FollowDecBlocked, 3.00);
	n.param<double>(         "FindSpeed",         FindSpeed, 0.10);
	n.param<double>(    "FindSpeedAngle",    FindSpeedAngle, 1.05);
	n.param<double>(     "FindGoalAngle",     FindGoalAngle, 0.15);
	n.param<double>(   "FindSpeedFollow",   FindSpeedFollow, 0.17);
	n.param<double>(     "AvoidWaitTime",     AvoidWaitTime, 3.00);
	n.param<double>(        "AvoidSpeed",        AvoidSpeed, 0.09);
	n.param<double>(    "AvoidGoalAngle",    AvoidGoalAngle, 0.18);

	// Create sub-controllers
	_segmentStart  = new cSegmentStart ( StartCheckRange, StartSpeed);
	_segmentFollow = new cSegmentFollow( FollowExtraRadius, FollowSpeed, FollowOffset, FollowDecBlocked);
	_segmentFind   = new cSegmentFind  ( FindSpeed, FindSpeedAngle, FindGoalAngle, FindSpeedFollow);
	_avoidObstacle = new cAvoidObstacle( AvoidWaitTime, AvoidSpeed, AvoidGoalAngle);
	_plantQuality  = new cPlantQuality ( );
	_freeRun       = new cFreeRun      ( );
	
	// Init sub-controllers
	_segmentStart->Create();
	_segmentFollow->Create();
	_segmentFind->Create();
	_avoidObstacle->Create();
	_plantQuality->Create( &_rosCom);
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
	// Get VDState
	flarb_msgs::VDState vdState;
	_rosCom.GetVDState( vdState);


	// We received a correct string, create a new cInputString obj
	if( str[0] == 'S')
	{
		_inputString = cInputString( str);
		_state       = STATE_SEGMENT_START;
		_segmentFollow->Reinit( vdState);
	}
	// We start in freerun mode
	else if( str[0] == 'F')
	{
		_state = STATE_FREERUN;
		_freeRun->Reinit( vdState);
	}
	// Debug STATE_DEBUG_FIND
	else if( str[0] == 'G')
	{
		_inputString = cInputString( "S - 1L - F");
		_state       = STATE_DEBUG_FIND;
		_segmentFind->Reinit( vdState, _inputString);
	}
	// Debug STATE_DEBUG_AVOID
	else if( str[0] == 'H')
	{
		_blocked = true;
		_state   = STATE_DEBUG_AVOID;
		_avoidObstacle->Reinit( vdState);
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
	// If we are in the init state, or even stopped :O, quit right away
	if( _state == STATE_INIT || _state == STATE_STOPPED)
		return;

	// Vars
	tVector output;
	flarb_msgs::VDState vdState;
	enum SUBRETURN ret;
	bool exec = false;
	int tries = 0;

	_rosCom.GetVDState( vdState);

	//
	while( !exec && tries < 5)
	{
		tries++;

		// AvoidObstacle first
		if( _blocked == true)
		{
			ret = _avoidObstacle->Execute( output, vdState, map);
			if( ret == RET_SUCCESS)
				exec = true;
			else // Avoidance is completed
				_blocked = false;
			continue;
		}

		// Execute the current sub-controller
		switch( _state)
		{
			case STATE_DEBUG_FIND:
				ret = _segmentFind->Execute( output, vdState, map);
				if( ret == RET_SUCCESS)
					exec = true;
				else
				{
					cout << "[controller] DEBUG_FIND done" << endl;
					exec   = true;
					_state = STATE_STOPPED;
				}
				break;


			case STATE_DEBUG_AVOID: // If we reach this, AvoidObstacle is already completed
				exec = true;
				cout << "[controller] DEBUG_AVOID done" << endl;
				_state = STATE_STOPPED;
				break;


			case STATE_FREERUN:
				ret = _freeRun->Execute( output, vdState, map);
				if( ret == RET_SUCCESS)
					exec = true;
				else
				{
					cout << "[controller] FreeRun: Path is blocked!" << endl;
					_avoidObstacle->Reinit( vdState);
					_blocked = true;
				}
				break;


			case STATE_SEGMENT_START:
				ret = _segmentStart->Execute( output, vdState, map);

				if( ret == RET_SUCCESS)
					exec = true;
				else
				{
					cout << "[controller] Switching to segment follow1" << endl;
					_segmentFollow->Reinit( vdState);
					_plantQuality->Reinit( vdState);
					_state = STATE_SEGMENT_FOLLOW;
				}
				break;


			case STATE_SEGMENT_FOLLOW:
				ret = _segmentFollow->Execute( output, vdState, map);
				if( ret == RET_SUCCESS)
				{
					_plantQuality->Execute( map);
					exec = true;
				}
				else if ( ret == RET_NEXT)
				{
					if( _inputString.IsNextSegment())
					{
						cout << "[controller] Switching to segment find" << endl;
						_segmentFind->Reinit( vdState, _inputString);
						_state = STATE_SEGMENT_FIND;
					}
					else
					{
						cout << "[controller] Assignment complete!" << endl;
						exec   = true;
						_state = STATE_STOPPED;
					}
				}
				else
				{
					cout << "[controller] SegFollow: Path is blocked!" << endl;
					_avoidObstacle->Reinit( vdState);
					_blocked = true;
				}
				break;


			case STATE_SEGMENT_FIND:
				ret = _segmentFind->Execute( output, vdState, map);
				if( ret == RET_SUCCESS)
					exec = true;
				else if ( ret == RET_NEXT)
				{
					cout << "[controller] Switching to segment follow2" << endl;
					_segmentFollow->Reinit( vdState);
					_plantQuality->Reinit( vdState);
					_inputString.currentSegment++;
					_state = STATE_SEGMENT_FOLLOW;
				}
				else
				{
					cout << "[controller] SegFind: 180 degree turning" << endl;
					_avoidObstacle->Reinit( vdState);
					_blocked = true;
				}
				break;


			default:
				cerr << "[controller] ERROR!! Unknown state" << endl;
				exec = true;
				break;
		}
	}

	// Tries reached 5, oh dear..
	if( tries >= 5)
	{
		cerr << "[controller] While-looped exceeded 5 loops, oh dear.." << endl;
		_state = STATE_STOPPED;
		return;
	}

	// We have a filled message by now, publish it
	flarb_msgs::WaypointVector msg;
	msg.x = output.getX();
	msg.y = output.getY();
	msg.QuickTurn = _blocked;
	_rosCom.PublishWaypoint( msg);
}

