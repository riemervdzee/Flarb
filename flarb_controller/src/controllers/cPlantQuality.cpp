// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include "ros/ros.h"
#include "flarb_controller/controllers/cPlantQuality.h"

// Functions executed at the beginning and end of the Application
bool cPlantQuality::Create( cRosCom *_ros)
{
	_rosCom = _ros;
	return true;
}

void cPlantQuality::Destroy()
{

}


// Gets called when we switch to the SegmentFollow controller
void cPlantQuality::Reinit( const flarb_msgs::VDState &state)
{
	_previousLeft  = false;
	_previousRight = false;
}


// This is a stripped Execute function, as we only need to know the surrounding
// The sub-controller doesn't influence the surrounding at all..
void cPlantQuality::Execute( cMap &map)
{
	// The boundingBoxes
	tBoundingBox bbl ( tVector( -0.5f, 0.0f), tVector( 0.0f, 0.1f));
	tBoundingBox bbr ( tVector(  0.0f, 0.0f), tVector( 0.5f, 0.1f));

	// Check both directions
	bool resultLeft  = map.CheckIntersectionRegion( bbl);
	bool resultRight = map.CheckIntersectionRegion( bbr);

	// Vars
	bool checkLeft = false, checkRight = false;

	// Check if the Left-state is changed, if true set checkLeft to true
	if( _previousLeft != resultLeft)
	{
		_previousLeft = resultLeft;
		if( resultLeft)
			checkLeft = true;
	}

	// Same but for Right
	if( _previousRight != resultRight)
	{
		_previousRight = resultRight;
		if( resultRight)
			checkRight = true;
	}

	if( checkLeft || checkRight)
	{
		_rosCom->PublishPlantQualityRequest( checkLeft, checkRight);
	}
}
