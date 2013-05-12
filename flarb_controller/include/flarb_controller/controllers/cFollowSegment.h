#ifndef CLASS_FOLLOW_SEGMENT_H
#define CLASS_FOLLOW_SEGMENT_H

#include <vector>

#include "flarb_controller/types/tVector.h"
#include "flarb_controller/types/tMatrix.h"

#include "flarb_mapbuilder/Map.h"


/*
 * Main controller class of the example node
 */
class cFollowSegment
{
public:
	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Passes reference of "vector", is used as output
	// Executes the FollowSegment sub-controller based on the rest of the arguments
	// TODO maybe more parameters?
	bool Execute( tVector &vector, const flarb_mapbuilder::Map &map);

private:
	std::vector<tVector> _WaypointAttempts;
};

#endif // CLASS_FOLLOW_SEGMENT_H
