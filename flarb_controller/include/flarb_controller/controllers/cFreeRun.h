#ifndef CLASS_FREERUN_H
#define CLASS_FREERUN_H

#include "flarb_controller/types/tVector.h"
#include "flarb_controller/types/tMatrix.h"

#include "flarb_msgs/VDState.h"
#include "flarb_controller/Config.h"
#include "flarb_controller/cMap.h"
#include "flarb_controller/cController.h"


/*
 * Our FreeRun sub-controller, which just tries to drive as much as possible
 */
class cFreeRun
{
public:
	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Gets called when we switch to the FreeRun controller
	void Reinit( const flarb_msgs::VDState &state);

	// Passes reference of "vector", is used as output
	// Executes the FollowSegment sub-controller based on the rest of the arguments
	enum SUBRETURN Execute( tVector &output, const flarb_msgs::VDState &state, cMap &map);
};

#endif // CLASS_FREERUN_H
