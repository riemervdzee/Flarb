#ifndef CLASS_AVOID_OBSTACLE_H
#define CLASS_AVOID_OBSTACLE_H

#include "flarb_controller/types/tVector.h"
#include "flarb_controller/types/tMatrix.h"

#include "flarb_msgs/VDState.h"
#include "flarb_controller/Config.h"
#include "flarb_controller/cMap.h"
#include "flarb_controller/cController.h"

enum AVOIDOBJ_STATES {
	AVOIDOBJ_WAIT1,
	AVOIDOBJ_TURNAXIS,
	AVOIDOBJ_WAIT2,
};

/*
 * AvoidObstacle sub-controller
 */
class cAvoidObstacle
{
public:
	// C-tor
	cAvoidObstacle() : _StopCount( 0), _direction(0.0f) {}

	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Gets called when we switch to the AvoidObstacle controller
	void Reinit( const flarb_msgs::VDState &state);

	// Passes reference of "vector", is used as output
	// Executes the AvoidObstacle sub-controller based on the rest of the arguments
	enum SUBRETURN Execute( tVector &output, const flarb_msgs::VDState &state, cMap &map);

private:
	// Our private vars, see source for info
	int _StopCount;
	float _direction;
};

#endif // CLASS_AVOID_OBSTACLE_H
