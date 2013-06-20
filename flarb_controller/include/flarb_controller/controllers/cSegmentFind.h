#ifndef CLASS_SEGMENT_FIND_H
#define CLASS_SEGMENT_FIND_H

#include "flarb_controller/types/tVector.h"
#include "flarb_controller/types/tMatrix.h"

#include "flarb_msgs/State.h"
#include "flarb_controller/Config.h"
#include "flarb_controller/cMap.h"
#include "flarb_controller/cController.h"


/*
 * Finds the next segment
 */
class cSegmentFind
{
public:
	// C-tor
	cSegmentFind(): _direction(0) {}

	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Gets called when we switch to the SegmentFind controller
	void Reinit( const flarb_msgs::State &state, const cInputString &str);

	// Passes reference of "vector", is used as output
	// Executes the FindSegment sub-controller based on the rest of the arguments
	enum SUBRETURN Execute( tVector &output, const flarb_msgs::State &state, cMap &map);

private:
	//
	float    _direction;
	sSegment _segment;
};

#endif // CLASS_SEGMENT_FIND_H
