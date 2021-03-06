#ifndef CLASS_FIND_SEGMENT_H
#define CLASS_FIND_SEGMENT_H

#include "flarb_img_controller/types/tVector.h"
#include "flarb_img_controller/types/tMatrix.h"

#include "flarb_img_controller/cImage.h"


/*
 * Main controller class of the example node
 */
class cFindSegment
{
public:
	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Passes reference of "vector", is used as output
	// Executes the FindSegment sub-controller based on the rest of the arguments
	// TODO maybe more parameters?
	bool Execute( tVector &vector, const cImage &image);

private:
};

#endif // CLASS_FIND_SEGMENT_H
