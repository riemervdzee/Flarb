// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include "ros/ros.h"
#include "flarb_controller/cAlgorithm.h"
#include "flarb_controller/controllers/cFollowSegment.h"
using namespace std;

// Functions executed at the beginning and end of the Application
bool cFollowSegment::Create()
{
	_WaypointAttempts.reserve(10);
	return true;
}

void cFollowSegment::Destroy()
{

}

// Passes reference of "vector", is used as output
// Executes the FollowSegment sub-controller based on the rest of the arguments
// TODO maybe more parameters?
bool cFollowSegment::Execute( tVector &vector, const flarb_mapbuilder::Map &map)
{
	// Our first attempt
	_WaypointAttempts.clear();
	_WaypointAttempts.push_back( tVector( 0.0f, 0.2f));

	// We don't use pop_front, as it degrades performance
	// Only downside of this approach is, the amount of mem we might use
	// TODO instead of using a std::vector, maybe a queue?
	unsigned int index = 0;
	bool result = false;

	while(!result && _WaypointAttempts.size() > index)
	{
		bool collision_found = false;
		tVector current = _WaypointAttempts[index];

		for( unsigned int i = 0; i < map.list.size(); i++)
		{
			flarb_mapbuilder::Object obj = map.list[i];
			tVector c = tVector( obj.x, obj.y);
			tVector p3;

			if( cAlgorithm::IntersectCircle( tVector(), current, c, obj.radius, p3))
			{
				collision_found = true;

				// TODO, check for uniqueness before pushing? "i"
				tVector p1, p2;
				cAlgorithm::GetOuterPoints( tVector(), c, obj.radius, p1, p2);
				_WaypointAttempts.push_back( p1);
				_WaypointAttempts.push_back( p2);
				_WaypointAttempts.push_back( p3);
				
			}
		}

		// We got a winner? otherwise proceed
		if(!collision_found) 
			result = true;
		else
			index++;
	}

	if( result)
		vector = _WaypointAttempts[index];

	return true;
}
