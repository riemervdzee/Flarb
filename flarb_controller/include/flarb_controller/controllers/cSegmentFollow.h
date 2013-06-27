#ifndef CLASS_SEGMENT_FOLLOW_H
#define CLASS_SEGMENT_FOLLOW_H

#include "flarb_controller/types/tVector.h"
#include "flarb_controller/types/tMatrix.h"

#include "flarb_msgs/VDState.h"
#include "flarb_controller/Config.h"
#include "flarb_controller/cMap.h"
#include "flarb_controller/cController.h"


/*
 * Follows the current segment
 */
class cSegmentFollow
{
public:
	// C-tor
	cSegmentFollow(float Radius, float Speed, float Offset, float DecBlocked, bool FollowVersionTwo, bool FollowVTRecheck) :
		_ParamRadius( Radius), _ParamSpeed( Speed), _ParamOffset( Offset),
		_ParamDecBlocked( DecBlocked), _ParamFollowVersionTwo( FollowVersionTwo),
		_ParamFollowVTRecheck( FollowVTRecheck), _StartDistance(0.0f), _GoalDistance( 0.0f) {}

	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	// Gets called when we switch to the SegmentFollow controller
	void Reinit( const flarb_msgs::VDState &state);

	// Passes reference of "vector", is used as output
	// Executes the SegmentFollow sub-controller based on the rest of the arguments
	enum SUBRETURN Execute( tVector &output, const flarb_msgs::VDState &state, cMap &map);

private:

	// Parameterized values
	float _ParamRadius;
	float _ParamSpeed;
	float _ParamOffset;
	float _ParamDecBlocked;
	bool  _ParamFollowVersionTwo;
	bool  _ParamFollowVTRecheck;

	// We try to get atleast past this goal distance after checking for segment-end
	float _StartDistance;
	float _GoalDistance;
};

#endif // CLASS_SEGMENT_FOLLOW_H
