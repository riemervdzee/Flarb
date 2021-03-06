#ifndef CLASS_PLANTQUALITY_H
#define CLASS_PLANTQUALITY_H

#include "flarb_controller/types/tVector.h"
#include "flarb_controller/types/tMatrix.h"

#include "flarb_controller/Config.h"
#include "flarb_controller/cMap.h"
#include "flarb_controller/cController.h"

class cRosCom;

/*
 * PlantQuality Controller, signals whether we see a new object at both sides
 * Gets called when in the same state as SegmentFollow
 */
class cPlantQuality
{
public:
	// Functions executed at the beginning and end of the Application
	bool Create( cRosCom *_ros);
	void Destroy();

	// Gets called when we switch to the SegmentFollow controller
	void Reinit( const flarb_msgs::VDState &state);

	// This is a stripped Execute function, as we only need to know the surrounding
	// The sub-controller doesn't influence the surrounding at all..
	void Execute( cMap &map);

private:
	// rosCom refference
	cRosCom *_rosCom;

	//
	bool _previousLeft;
	bool _previousRight;
};

#endif // CLASS_PLANTQUALITY_H
