#ifndef CLASS_FOLLOW_SEGMENT_H
#define CLASS_FOLLOW_SEGMENT_H

#include "flarb_controller/WaypointVector.h"
#include "flarb_controller/cImage.h"


/*
 * Main controller class of the example node
 */
class cFollowSegment
{
public:
	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Passes reference of "msg", is used as output
	// Executes the FollowSegment sub-controller based on the rest of the arguments
	// TODO maybe more parameters?
	bool Execute( flarb_controller::WaypointVector &msg, const cImage &image);

private:
};

#endif // CLASS_FOLLOW_SEGMENT_H