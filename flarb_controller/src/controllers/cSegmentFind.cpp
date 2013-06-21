// Include order: cppstd, ROS, Boost, own-module includes
#include <cmath>
#include <iostream>
#include "ros/ros.h"
#include "flarb_controller/controllers/cSegmentFind.h"
using namespace std;


// Functions executed at the beginning and end of the Application
bool cSegmentFind::Create()
{
	return true;
}

void cSegmentFind::Destroy()
{

}

// Gets called when we switch to the SegmentFind controller
void cSegmentFind::Reinit( const flarb_msgs::State &state, const cInputString &str)
{
	// Set vars
	_segment = str.segments[ str.currentSegment];
	_GoalDir = state.response.axisZ;

	// Depending on the direction, set the goal dir
	switch( _segment.rowdir)
	{
		// We are cheating here, we just say it is blocked so AvoidObstacle gets called
		case DIR_RETURN:
			_state = SEGFIND_TURNAXIS1;
			return;

		case DIR_LEFT:
			_GoalDir += (M_PI/2);
			break;

		case DIR_RIGHT:
			_GoalDir -= (M_PI/2);
			break;
	}

	// Clamp: 0 <= _direction <= 2xPI
	if( _GoalDir > (2*M_PI))
		_GoalDir -= (2*M_PI);
	else if( _GoalDir < 0)
		_GoalDir += (2*M_PI);

	// We need to turn first
	_state = SEGFIND_TURN1;
}

// Passes reference of "msg", is used as output
// Executes the SegmentFind sub-controller based on the rest of the arguments
enum SUBRETURN cSegmentFind::Execute( tVector &output, const flarb_msgs::State &state, cMap &map)
{
	//
	enum SUBRETURN ret;
	bool exec = false;
	int tries = 0;
	output = tVector();

	while( !exec && tries < 5)
	{
		tries++;

		switch( _state)
		{
			case SEGFIND_TURNAXIS1:
				_state = SEGFIND_TURNAXIS2;
				ret    = RET_BLOCKED;
				exec   = true;
				break;

			case SEGFIND_TURNAXIS2:
				ret  = RET_NEXT;
				exec = true;
				break;


			case SEGFIND_TURN1:
				ret = Turn( output, state, map);
				if( ret == RET_SUCCESS)
					exec = true;
				else
				{
					_state = SEGFIND_DRIVESTRAIGHT;
					cout << "[SegmentFind] Turn1 completed" << endl;
				}
				break;


			case SEGFIND_DRIVESTRAIGHT:
				ret = Straight( output, state, map);
				if( ret == RET_SUCCESS)
					exec = true;
				else
				{
					_state = SEGFIND_TURN2;
					cout << "[SegmentFind] Drive straight completed" << endl;
				}
				break;


			case SEGFIND_TURN2:
				ret = Turn( output, state, map);
				if( ret == RET_SUCCESS)
					exec = true;
				else
				{
					exec = true;
					cout << "[SegmentFind] Turn2 completed" << endl;
				}
				break;
		}
	}

	return ret;
}

enum SUBRETURN cSegmentFind::Turn( tVector &output, const flarb_msgs::State &state, cMap &map)
{
	// Get the difference between the goal-angle and the current angle
	float difference = _GoalDir - state.response.axisZ;

	// Clamp: 0 <= _direction <= 2xPI
	if( difference > (2*M_PI))
		difference -= (2*M_PI);
	else if( difference < 0)
		difference += (2*M_PI);

	// Is the difference quite small? advance
	if( difference > -0.09 && difference < 0.09)
		return RET_NEXT;

	// The max difference is 45 degrees
	if( difference > (M_PI/4))
		difference =  (M_PI/4);
	else if( difference < -(M_PI/4))
		difference = -(M_PI/4);

	// Add quarter turn to the left, so the dir points straight ahead-ish
	difference += (M_PI/2);
	tVector vec (cos( difference), sin( difference));

	float result = map.FindFreePath( FLARB_EXTRA_RADIUS, vec, output, false);
	// TODO, do something with result
	return RET_SUCCESS;
}

enum SUBRETURN cSegmentFind::Straight( tVector &output, const flarb_msgs::State &state, cMap &map)
{
	// TODO skip rows
	_state = SEGFIND_TURN2;

	// Depending on the direction, set the goal dir
	switch( _segment.rowdir)
	{
		case DIR_LEFT:
			_GoalDir += (M_PI/2);
			break;

		case DIR_RIGHT:
			_GoalDir -= (M_PI/2);
			break;

		default:
			cout << "[ERROR] cSegmentFind-straight error 1" << endl;
	}

	// Clamp: 0 <= _direction <= 2xPI
	if( _GoalDir > (2*M_PI))
		_GoalDir -= (2*M_PI);
	else if( _GoalDir < 0)
		_GoalDir += (2*M_PI);

	return RET_NEXT;
}
