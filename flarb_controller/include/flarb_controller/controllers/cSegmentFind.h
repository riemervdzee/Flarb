#ifndef CLASS_SEGMENT_FIND_H
#define CLASS_SEGMENT_FIND_H

#include "flarb_controller/types/tVector.h"
#include "flarb_controller/types/tMatrix.h"
#include "flarb_controller/types/tBoundingBox.h"

#include "flarb_msgs/VDState.h"
#include "flarb_controller/Config.h"
#include "flarb_controller/cMap.h"
#include "flarb_controller/cController.h"

enum SEGFIND_STATES {
	SEGFIND_TURN1,
	SEGFIND_DRIVESTRAIGHT,
	SEGFIND_TURN2,
	SEGFIND_GETINROW,
	SEGFIND_TURNAXIS,
};

/*
 * Finds the next segment
 */
class cSegmentFind
{
public:
	// C-tor
	cSegmentFind(): _GoalDir(0) {}

	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Gets called when we switch to the SegmentFind controller
	void Reinit( const flarb_msgs::VDState &state, const cInputString &str);

	// Passes reference of "vector", is used as output
	// Executes the FindSegment sub-controller based on the rest of the arguments
	enum SUBRETURN Execute( tVector &output, const flarb_msgs::VDState &state, cMap &map);

private:
	// To simplify the Execute function, we have seperate func
	enum SUBRETURN Turn    ( tVector &output, const flarb_msgs::VDState &state, cMap &map);
	enum SUBRETURN Straight( tVector &output, const flarb_msgs::VDState &state, cMap &map);
	enum SUBRETURN GetInRow( tVector &output, const flarb_msgs::VDState &state, cMap &map);


	// The current segment we execute
	sSegment _segment;

	// Current state in solving the segment
	enum SEGFIND_STATES _state;

	// The goaldir, meaning depends on state
	float _GoalDir;

	// For skipping rows
	int          _skip;     // Amount we still need to skip
	bool         _previous; // Whether the previous frame was blocked or not
	tBoundingBox _bb;       // BoundingBox where we need to check at
};

#endif // CLASS_SEGMENT_FIND_H
