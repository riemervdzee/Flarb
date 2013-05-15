#include <cmath>
#include <algorithm>
#include <iostream>
#include "flarb_controller/cMap.h"
using namespace std;


/*
 * Sets the map list to use for the upcoming functions
 */
void cMap::setMapList( const flarb_mapbuilder::MapList &map)
{
	_map = &map;
}


/*
 * Helpers for cMap::FindFreePath
 */
static tVector CompareTo;
static bool compare_tVector( const tVector &first, const tVector &second)
{
	return CompareTo.Dot( first) > CompareTo.Dot( second);
}


/*
 * Checks whether "input" is accessable. The ending result is put in "output"
 * Returns the dotproduct between input and output. range is 1 to -1
 * 1 means input = output. 0 = 90 degrees difference, -1 = 180 degrees difference
 * FREEPATH_NO_SOLUTION = special case, output = {0,0} no solution possible
 */
float cMap::FindFreePath( const float protection_margin, const tVector &input, tVector &output)
{
	// Clear containers. Vars used in the upcoming while loop
	_WaypointAttempts.clear();
	_ObjectsCollided.clear();
	bool  result = false; // positive result?
	float inputLength = input.Length();
	tVector currentAttempt;

	// Push our first attempt (input), set the vector we should compare to
	_WaypointAttempts.push_back( input);
	CompareTo = input;

	// When there is no result yet, when it ain't empty or too large
	while(!result && !_WaypointAttempts.empty() && _WaypointAttempts.size() < 50)
	{
		// Var to tell when we got a collision, gets the attempt with the lowest score
		bool collision_found = false;

		// Gets the vector which corosponds the most with input
		currentAttempt = _WaypointAttempts.front();
		_WaypointAttempts.pop_front();

		// Go through all objects, check for collisions
		for( unsigned int i = 0; i < _map->list.size(); i++)
		{
			flarb_mapbuilder::Object obj = _map->list[i];
			tVector c = tVector( obj.x, obj.y);
			tVector p1;

			// Check for collision
			float r = (obj.radius + protection_margin);
			if( IntersectCircle( tVector(), currentAttempt, c, r, p1))
			{
				collision_found = true;

				// Shrink p1
				// TODO fix?
				//p1 *= 0.9;
				//_WaypointAttempts.push_back( p1);

				// Check for uniquenes
				bool unique = true;
				for( unsigned int j = 0; j < _ObjectsCollided.size(); j++)
				{
					if( _ObjectsCollided[j] == i)
					{
						unique = false;
						break;
					}
				}

				// If unique, gather the outer points as well
				// Otherwise these points are/were already in the list
				if( unique)
				{
					// Something extra to prevent future collisions!
					float radius = obj.radius + protection_margin + 0.001f;
					tVector p2, p3;

					// Execute
					GetOuterPoints( tVector(), c, radius, p2, p3);

					// Resize p2 and p3 to the length of the input and insert
					// TODO only if p1 is fixed!
					//p2.setLength( inputLength);
					//p3.setLength( inputLength);
					_WaypointAttempts.push_back( p2);
					_WaypointAttempts.push_back( p3);

					_ObjectsCollided.push_back( i);
				}
			}
		}

		// We got a winner? Otherwise sort and proceed
		if(!collision_found)
			result = true;
		else
			_WaypointAttempts.sort( compare_tVector);
	}


	// Do we got a result?
	if( result)
	{
		output = currentAttempt;

		// The return val is the dot-product/diff of the normalized vectors
		tVector inputN  = input * (1 / inputLength);
		tVector outputN = output;
		outputN.Normalize();
		return inputN.Dot( outputN);
	}
	// Otherwise, set the values to 0
	else
	{
		output = tVector();
		return FREEPATH_NO_SOLUTION;
	}
}

#if 0
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

#else
// TODO bit messy
bool cMap::IntersectCircle( tVector l1, tVector l2, tVector circle,
	float radius, tVector &result)
{
	// Empty the resulting vector
	result = tVector();

	// Translate coordinates so l1 = {0,0)
	tVector l  = l2 - l1;
	tVector cs = circle - l1;

	// Get discriminant
	float a = l.Dot( l);
	float b = 2 * l.Dot( cs);
	float c = cs.Dot( cs) -  (radius * radius);
	float d = (b * b) - (4 * a * c);

	// If d is negative, no real solution exists. This means that the infinite
	// line formed by lineStart and lineEnd doesn't collide with the circle
	if( d < 0)
		return false;

	// Although if d == 0, only one solution exists. but chances are low due
	// floating point arithmetics. If it is actually 0, it won't have any effect
	// Just try to look for the two possible solutions. Note that these two
	// solutions are the points of the infinite line, not the segment we want
	float dsqrt=sqrt(d);
	float div = 2 * a;
	float t1 = (-b - dsqrt) / div;
	float t2 = (-b + dsqrt) / div;

	// The two results
	// TODO check correctness
	float x1 = (-1 * l.getX() * t1);
	float y1 = (-1 * l.getY() * t1);
	float x2 = ( 1 * l.getX() * t2);
	float y2 = ( 1 * l.getY() * t2);

	// To check whether they lie on our segment, do a bounding box test
	float minX = 0, maxX = l.getX();
	float minY = 0, maxY = l.getY();

	if( maxX < 0)
		std::swap( minX, maxX);
	if( maxY< 0)
		std::swap( minY, maxY);

	// Check whether one of these two points actually lies in the segment
	bool r1 = false, r2 = false;

	if (x1 >= minX && x1 <= maxX && y1 >= minY && y1 <= maxY)
		r1 = true;
	if (x2 >= minX && x2 <= maxX && y2 >= minY && y2 <= maxY)
		r2 = true;

	// If both ain't in the segment, the segment ain't coliding with the circle
	if( r1 == false && r2 == false)
		return false;
	// If either r1 or r2 is true (exclusive), return the retranslated point
	else if( r1 == true && r2 == false)
	{
		result = l1 + tVector( x1, y1);
		return true;
	}
	else if( r1 == false && r2 == true)
	{
		result = l1 + tVector( x2, y2);
		return true;
	}
	// Both lie on the segment, pick the one with the smallest length
	// smallestlength = point which we collide first. coming from l1
	else
	{
		// We calculate the squared length
		float len1 = (x1*x1) + (y1*y1);
		float len2 = (x2*x2) + (y2*y2);

		if( len1 < len2)
			result = l1 + tVector( x1, y1);
		else
			result = l1 + tVector( x2, y2);

		return true;
	}
}
#endif

/*
 * Having a point (lineStart) and a circle (circle + radius), there are two
 * lines from this point where there is only one intersection with the circle.
 * These are also called tangent-lines, but have their base as lineStart
 * This function puts the colisionpoints of these two lines into p1 and p2
 *
 * See the documentation for details
 */
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
