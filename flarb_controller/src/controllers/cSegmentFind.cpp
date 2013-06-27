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
void cSegmentFind::Reinit( const flarb_msgs::VDState &state, const cInputString &str)
{
	// Set vars
	_segment = str.segments[ str.currentSegment];
	_GoalDir = state.response.axisZ;

	// Depending on the direction, set the goal dir
	switch( _segment.rowdir)
	{
		// We are cheating here, we just say it is blocked so AvoidObstacle gets called
		case DIR_RETURN:
			_state = SEGFIND_TURNAXIS;
			return;

		case DIR_LEFT:
			_GoalDir += (M_PI/2);
			_bb = tBoundingBox( tVector( -0.5f, 0.0f), tVector( 0.0f, 0.1f));
			break;

		case DIR_RIGHT:
			_GoalDir -= (M_PI/2);
			_bb = tBoundingBox( tVector( 0.0f, 0.0f), tVector( 0.5f, 0.1f));
			break;
	}

	// Clamp: 0 <= _direction <= 2xPI
	if( _GoalDir > (2*M_PI))
		_GoalDir -= (2*M_PI);
	else if( _GoalDir < 0)
		_GoalDir += (2*M_PI);

	// We need to turn first
	_state = SEGFIND_TURN1;

	// The amount of rows we need to skip
	_skip = _segment.row_count - 1;
}


// Passes reference of "msg", is used as output
// Executes the SegmentFind sub-controller based on the rest of the arguments
enum SUBRETURN cSegmentFind::Execute( tVector &output, const flarb_msgs::VDState &state, cMap &map)
{
	// Vars
	enum SUBRETURN ret = RET_BLOCKED;
	bool exec = false;
	int tries = 0;
	output    = tVector();

	while( !exec && tries < 5)
	{
		tries++;

		switch( _state)
		{
			case SEGFIND_TURNAXIS:
				_state = SEGFIND_GETINROW;;
				ret    = RET_BLOCKED;
				exec   = true;
				break;


			case SEGFIND_TURN1:
				ret = Turn( output, state, map);
				if( ret == RET_SUCCESS)
				{
					exec = true;
				}
				else
				{
					_previous = map.CheckIntersectionRegion( _bb);
					_state = SEGFIND_DRIVESTRAIGHT;
					cout << "[SegmentFind] Turn1 completed" << endl;
				}
				break;


			case SEGFIND_DRIVESTRAIGHT:
				ret = Straight( output, state, map);
				if( ret == RET_SUCCESS)
				{
					exec = true;
				}
				else
				{
					_state = SEGFIND_TURN2;
					cout << "[SegmentFind] Drive straight completed" << endl;
				}
				break;


			case SEGFIND_TURN2:
				ret = Turn( output, state, map);
				if( ret == RET_SUCCESS)
				{
					exec = true;
				}
				else
				{
					_state = SEGFIND_GETINROW;
					cout << "[SegmentFind] Turn2 completed" << endl;
				}
				break;


			case SEGFIND_GETINROW:
				ret = GetInRow( output, state, map);
				if( ret == RET_SUCCESS)
				{
					exec = true;
				}
				else
				{
					exec = true;
					cout << "[SegmentFind] GetInRow completed" << endl;
				}
				break;
		}
	}

	// Tries reached 5, oh dear..
	if( tries >= 5)
	{
		cerr << "[SegmentFind] While-looped exceeded 5 loops, oh dear.." << endl;
		ret = RET_BLOCKED;
	}

	return ret;
}


// Turn 90 degrees routine
enum SUBRETURN cSegmentFind::Turn( tVector &output, const flarb_msgs::VDState &state, cMap &map)
{
	// Get the difference between the goal-angle and the current angle
	float difference = _GoalDir - state.response.axisZ;

	// Clamp: -PI <= _direction <= +PI
	if( difference > M_PI)
		difference -= (2*M_PI);
	else if( difference < -M_PI)
		difference += (2*M_PI);

	// Is the difference quite small? advance
	if( difference > -_ParamGoalAngle && difference < _ParamGoalAngle)
		return RET_NEXT;


	// Strengthen difference, to get a better turn
	difference *= 4;

	// The max difference we turn is 50 degrees
	if( difference > _ParamSpeedAngle)
		difference = _ParamSpeedAngle;
	else if( difference < -_ParamSpeedAngle)
		difference = -_ParamSpeedAngle;

	// Add quarter turn to the left, so the dir points straight ahead-ish
	difference += (M_PI/2);
	tVector vec (
		cos( difference) * _ParamSpeed,
		sin( difference) * _ParamSpeed);

	// TODO, do something with result?
	//float result = map.FindFreePath( FLARB_EXTRA_RADIUS, vec, output, false);
	map.FindFreePath( FLARB_EXTRA_RADIUS, vec, output, false);	
	return RET_SUCCESS;
}


// Drive straight routine, skipping any rows when neccesary
enum SUBRETURN cSegmentFind::Straight( tVector &output, const flarb_msgs::VDState &state, cMap &map)
{
	// Do we need to skip? check if we can skip one right now
	if( _skip > 0)
	{
		bool current = map.CheckIntersectionRegion( _bb);
		if (current != _previous)
		{
			if(!current)
				_skip--;
			_previous = current;
		}
	}

	// Do we still need to skip? drive straight ahead
	if( _skip > 0)
	{
		// Just drive straight ahead
		tVector direction = tVector( 0.0f, _ParamSpeed);
		map.FindFreePath( FLARB_EXTRA_RADIUS, direction, output, false);
		return RET_SUCCESS;
	}
	// All rows skipped, We can turn now
	else
	{

		// We need to turn again
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

		_state = SEGFIND_TURN2;
		return RET_NEXT;
	}
}


// Gets into a row, so SegmentFind can easily take over
enum SUBRETURN cSegmentFind::GetInRow( tVector &output, const flarb_msgs::VDState &state, cMap &map)
{
	// First check if there is room at both sides
	tBoundingBox b( tVector( -0.5f, 0), tVector( 0.5f, 0.3f));
	if(map.CheckIntersectionRegion( b))
	{
		return RET_NEXT;
	}

	// Just drive straight ahead
	tVector direction = tVector( 0.0f, _ParamSpeedFollow);
	map.FindFreePath( FLARB_EXTRA_RADIUS, direction, output, true);

	return RET_SUCCESS;
}
