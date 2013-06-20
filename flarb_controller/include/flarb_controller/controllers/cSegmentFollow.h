#ifndef CLASS_SEGMENT_FOLLOW_H
#define CLASS_SEGMENT_FOLLOW_H

#include "flarb_controller/types/tVector.h"
#include "flarb_controller/types/tMatrix.h"

#include "flarb_msgs/State.h"
#include "flarb_controller/Config.h"
#include "flarb_controller/cMap.h"
#include "flarb_controller/cController.h"


/*
 * Follows the current segment
 */
class cSegmentFollow
{
public:
	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Gets called when we switch to the SegmentFollow controller
	void Reinit( const flarb_msgs::State &state);

	// Passes reference of "vector", is used as output
	// Executes the SegmentFollow sub-controller based on the rest of the arguments
	enum SUBRETURN Execute( tVector &output, const flarb_msgs::State &state, cMap &map);
};

#endif // CLASS_SEGMENT_FOLLOW_H
