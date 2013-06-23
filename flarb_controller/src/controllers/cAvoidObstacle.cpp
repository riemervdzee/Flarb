// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "flarb_controller/controllers/cAvoidObstacle.h"
using namespace std;

// Functions executed at the beginning and end of the Application
bool cAvoidObstacle::Create()
{
	return true;
}

void cAvoidObstacle::Destroy()
{

}


// Gets called when we switch to the AvoidObstacle controller
void cAvoidObstacle::Reinit( const flarb_msgs::VDState &state)
{
	// If this is the first time the sub-controller gets executed in a series
	// Set the _count to a constant
	_Counter = 30 * FLARB_AVOID_WAITTIME;
	_GoalDir = state.response.axisZ + M_PI;
	_state   = AVOIDOBJ_WAIT1;

	// 0 <= _direction <= 2xPI
	if( _GoalDir > (2*M_PI))
		_GoalDir -= (2*M_PI);
}


// Passes reference of "msg", is used as output
// Executes the AvoidObstacle sub-controller based on the rest of the arguments
enum SUBRETURN cAvoidObstacle::Execute( tVector &output, const flarb_msgs::VDState &state, cMap &map)
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
			case AVOIDOBJ_WAIT1:
				ret = Wait( output, state, map);
				if( ret == RET_SUCCESS)
				{
					exec = true;
				}
				else if( ret == RET_NEXT)
				{
					_state = AVOIDOBJ_TURNAXIS;
					cout << "[AvoidObstacle] Wait1 completed" << endl;
				}
				break;


			case AVOIDOBJ_TURNAXIS:
				ret = Turn( output, state, map);
				if( ret == RET_SUCCESS)
				{
					exec = true;
				}
				else if( ret == RET_NEXT)
				{
					_state   = AVOIDOBJ_WAIT2;
					_Counter = 30 * FLARB_AVOID_WAITTIME;
					cout << "[AvoidObstacle] Turn completed" << endl;
				}
				break;


			case AVOIDOBJ_WAIT2:
				ret = Wait( output, state, map);
				if( ret == RET_SUCCESS)
				{
					exec = true;
				}
				else if( ret == RET_NEXT)
				{
					exec = true;
					cout << "[AvoidObstacle] Wait2 completed" << endl;
				}
				break;
		}
	}

	return ret;
}


// Wait routine
enum SUBRETURN cAvoidObstacle::Wait( tVector &output, const flarb_msgs::VDState &state, cMap &map)
{
	// As long as _StopCount > 0, try to stop
	if( _Counter > 0)
	{
		_Counter--;
		output = tVector();
		return RET_SUCCESS;
	}
	else
	{
		return RET_NEXT;
	}
}


// Turn routine
enum SUBRETURN cAvoidObstacle::Turn( tVector &output, const flarb_msgs::VDState &state, cMap &map)
{
	// Get the difference between the goal-angle and the current angle
	float difference = _GoalDir - state.response.axisZ;

	// Clamp: -PI <= _direction <= +PI
	if( difference > M_PI)
	{
		difference -= (2*M_PI);
	}
	else if( difference < -M_PI)
	{
		difference += (2*M_PI);
	}

	// Is the difference quite small?
	if( difference > -FLARB_AVOID_GOALANGLE && difference < FLARB_AVOID_GOALANGLE)
	{
		output = tVector();
		return RET_NEXT;
	}
	else
	{
		// Here is where we turn badly
		// TODO Base strength on difference?
		output = tVector( FLARB_AVOID_SPEED, 0);
		return RET_SUCCESS;
	}
}

