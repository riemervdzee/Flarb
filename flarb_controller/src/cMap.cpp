#include <cmath>
#include <iostream>
#include "flarb_controller/cMap.h"
using namespace std;

void cMap::setMapList( const flarb_mapbuilder::MapList &map)
{
	_map = &map;
}

// Helpers for cMap::FindFreePath
static tVector CompareTo;
static bool compare_tVector( const tVector &first, const tVector &second)
{
	return CompareTo.Dot( first) > CompareTo.Dot( second);
}


// TODO write explanation
float cMap::FindFreePath( const tVector &input, tVector &output)
{
	// Our first attempt
	_WaypointAttempts.clear();
	_WaypointAttempts.push_back( input);

	bool result = false;
	tVector current;

	while(!result && !_WaypointAttempts.empty() && _WaypointAttempts.size() < 50)
	{
		bool collision_found = false;
		current = _WaypointAttempts.front();
		_WaypointAttempts.pop_front();

		CompareTo = current;

		for( unsigned int i = 0; i < _map->list.size(); i++)
		{
			flarb_mapbuilder::Object obj = _map->list[i];
			tVector c = tVector( obj.x, obj.y);
			tVector p3;

			if( IntersectCircle( tVector(), current, c, obj.radius, p3))
			{
				collision_found = true;
				
				// Shrink p3
				// TODO fix?
				//p3 *= 0.9;

				// TODO, check for uniqueness before pushing? "i"
				tVector p1, p2;
				GetOuterPoints( tVector(), c, obj.radius + 0.001f, p1, p2);
				// TODO resize p1 and p2 to length of input
				_WaypointAttempts.push_back( p1);
				_WaypointAttempts.push_back( p2);
				//_WaypointAttempts.push_back( p3);

				// TODO should we break?
				//break;
			}
		}
		
		//cout << "collision_found " << collision_found << ", size " << _WaypointAttempts.size() << endl << endl;

		// We got a winner? otherwise proceed
		if(!collision_found)
		{
			result = true;
		}
		else
		{
			// Sort
			_WaypointAttempts.sort( compare_tVector);
		}
	}

	// Do we got a result? otherwise, set the values to 0
	if( result)
		output = current;
	else
		output = tVector();

	// The result is the dot/difference between the two vectors
	return input.Dot( output);
}


// Based on http://mathworld.wolfram.com/Circle-LineIntersection.html
// http://stackoverflow.com/questions/7060519/
// TODO cleanup
bool cMap::IntersectCircle( tVector lineStart, tVector lineEnd, 
		tVector circle, float radius, tVector &circleWhenHit)
{
	circleWhenHit = tVector();

	// find the closest point on the line segment to the center of the circle
	tVector line = lineEnd - lineStart;
	float lineLength = line.Length();
	tVector lineNorm = (1/lineLength)*line;
	tVector segmentToCircle = circle - lineStart;
	float closestPointOnSegment = segmentToCircle.Dot(line) / lineLength;

	// Special cases where the closest point happens to be the end points
	tVector closest;
	if (closestPointOnSegment < 0) closest = lineStart;
	else if (closestPointOnSegment > lineLength) closest = lineEnd;
	else closest = lineStart + closestPointOnSegment*lineNorm;

	// Find that distance.  If it is less than the radius, then we 
	// are within the circle
	tVector distanceFromClosest = circle - closest;
	float distanceFromClosestLength = distanceFromClosest.Length();
	if (distanceFromClosestLength > radius) return false;

	// So find the distance that places the intersection point right at 
	// the radius.  This is the center of the circle at the time of collision
	// and is different than the result from Doswa
	// TODO this is a tad faulty
	tVector offset = (radius - distanceFromClosestLength) *
					 ((1/distanceFromClosestLength)*distanceFromClosest);
	circleWhenHit = circle - offset;

	return true;
}


void cMap::GetOuterPoints( tVector lineStart, tVector circle, 
	float radius, tVector &p1, tVector &p2)
{
	tVector startToCircle = circle - lineStart;
	float ltot = startToCircle.Length();
	tVector N = startToCircle*(1/ltot);
	tVector P = N;
	P.Perpendicular();

	float r2 = radius*radius;
	float l1 = ltot - (r2 / ltot);
	float h  = (sqrt(ltot*ltot-r2) * radius) / ltot;

	tVector l1Vec = l1 * N;
	tVector hVec = h * P;
	p1 = l1Vec + hVec;
	p2 = l1Vec - hVec;
}
