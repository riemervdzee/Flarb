#ifndef CLASS_AVOID_OBSTACLE_H
#define CLASS_AVOID_OBSTACLE_H

#include "flarb_img_controller/types/tVector.h"
#include "flarb_img_controller/types/tMatrix.h"

#include "flarb_img_controller/cImage.h"


/*
 * Main controller class of the example node
 */
class cAvoidObstacle
{
public:
	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Passes reference of "vector", is used as output
	// Executes the AvoidObstacle sub-controller based on the rest of the arguments
	// TODO maybe more parameters?
	bool Execute( tVector &vector, const cImage &image);

private:
};

#endif // CLASS_AVOID_OBSTACLE_H
