#ifndef CLASS_FREERUN_H
#define CLASS_FREERUN_H

#include "flarb_controller/types/tVector.h"
#include "flarb_controller/types/tMatrix.h"

#include "flarb_VDMixer/State.h"
#include "flarb_controller/Config.h"
#include "flarb_controller/cMap.h"
#include "flarb_controller/cController.h"


/*
 * Main controller class of the example node
 */
class cFreeRun
{
public:
	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Passes reference of "vector", is used as output
	// Executes the FollowSegment sub-controller based on the rest of the arguments
	// TODO maybe more parameters?
	enum SUBRETURN Execute( tVector &output, const flarb_VDMixer::State &state, const cMap &map, bool reinit);
};

#endif // CLASS_FREERUN_H