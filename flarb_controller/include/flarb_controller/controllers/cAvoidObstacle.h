#ifndef CLASS_AVOID_OBSTACLE_H
#define CLASS_AVOID_OBSTACLE_H

#include "flarb_controller/WaypointVector.h"
#include "flarb_controller/cImage.h"


/*
 * Main controller class of the example node
 */
class cAvoidObstacle
{
public:
	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Passes reference of "msg", is used as output
	// Executes the AvoidObstacle sub-controller based on the rest of the arguments
	// TODO maybe more parameters?
	bool Execute( flarb_controller::WaypointVector &msg, const cImage &image);

private:
};

#endif // CLASS_AVOID_OBSTACLE_H