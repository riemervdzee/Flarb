#include <cmath>
#include <algorithm>
#include <iostream>
#include "flarb_controller/cMap.h"
using namespace std;


/*
 * Sets the map list to use for the upcoming functions
 */
void cMap::setMapList( const flarb_msgs::MapList &map)
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
 *
 *
 * float    protection_margin   Extra radius for each object
 * tVector  input               Input vector to try
 * tVector  output              The resulting vector, if any
 * bool     UseCollisionPoint   if true, use the collisionpoint as an attempt as well. 
 *                              use false if you want strict collision avoidance
 *
 */
float cMap::FindFreePath( const float protection_margin, const tVector &input, tVector &output, bool UseCollisionPoint)
{
	// Clear containers. Vars used in the upcoming while loop
	_WaypointAttempts.clear();
	_ObjectsCollided.clear();
	bool  result = false; // positive result?
	float inputLength = input.Length();
	float resizeLength = inputLength * 0.95f;
	tVector currentAttempt;

	// Push our first attempt (input), set the vector we should compare to
	_WaypointAttempts.push_back( input);
	CompareTo = input;

	// When there is no result yet, when it ain't empty or too large
	while(!result && !_WaypointAttempts.empty() && _WaypointAttempts.size() < 100)
	{
		// Var to tell when we got a collision, gets the attempt with the lowest score
		bool collision_found = false;

		// Gets the vector which corosponds the most with input
		currentAttempt = _WaypointAttempts.front();
		_WaypointAttempts.pop_front();

		//cout << "size " << _WaypointAttempts.size() + 1;
		//cout << ", Trying: " << currentAttempt.getX() << ", " << currentAttempt.getY() << endl;

		// Go through all objects, check for collisions
		for( unsigned int i = 0; i < _map->list.size(); i++)
		{
			flarb_msgs::Object obj = _map->list[i];
			tVector c = tVector( obj.x, obj.y);
			float r = (obj.radius + protection_margin);
			tVector p1;

			// Check for collision
			if( IntersectCircle( tVector(), currentAttempt, c, r, p1))
			{
				collision_found = true;

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

				//cout << "Collided with id" << i << endl;

				// If unique, gather the outer points as well
				// Otherwise these points are/were already in the list
				if( unique)
				{
					// Shrink p1
					if( UseCollisionPoint)
					{
						p1 *= 0.9;
						_WaypointAttempts.push_back( p1);
					}

					// Something extra to prevent future collisions!
					float radius = obj.radius + protection_margin + 0.002f;
					tVector p2, p3;

					// Execute
					GetOuterPoints( tVector(), c, radius, p2, p3);

					// Resize p2 and p3 to the length of the input and insert
					p2.setLength( resizeLength);
					p3.setLength( resizeLength);
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
		tVector inputN  = input  * (1 / inputLength);
		tVector outputN = output * (1 / inputLength);
		return inputN.Dot( outputN);
	}
	// Otherwise, set the values to 0
	else
	{
		output = tVector();
		cout << "Oh dear.. No result " << endl;
		return FREEPATH_NO_SOLUTION;
	}
}


/*
 * Tells whether a bbaa region with the map
 */
bool cMap::CheckIntersectionRegion( const tBoundingBox region)
{
	// Go through all objects, check for collisions
	for( unsigned int i = 0; i < _map->list.size(); i++)
	{
		flarb_msgs::Object obj = _map->list[i];
		tVector c = tVector( obj.x, obj.y);
		float r   = obj.radius;

		if( BoundingBoxCircleCollision( region, c, r))
			return true;
	}

	// We checked all objects, return fase
	return false;
}


/*
 * Tells us whether a finite line (l1, l2) intersects with the given circle and
 * radius. Puts the point of first contact in result
 * Based on http://mathworld.wolfram.com/Circle-LineIntersection.html
 */
bool cMap::IntersectCircle( tVector l1, tVector l2, tVector circle,
	float radius, tVector &result)
{
	// Empty the resulting vector
	result = tVector();

	// l = line end from {0,0}; cs line-beginning from circle
	tVector l  = l2 - l1;
	tVector cs = l1 - circle;

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
	// floating point arithmetics. If it is actually 0, it hardly has an effect
	// Just try to look for the two possible solutions. Note that we'll get two
	// Scalars. When t1 and t2 are in [0,1] range, it is a hit.
	// t1 is guaranteed to be first.
	float dsqrt=sqrt(d);
	float div = 2 * a;
	float t1 = (-b - dsqrt) / div;
	float t2 = (-b + dsqrt) / div;

	// If t1 is in the 0 >= t1 >= 1 range, it is a hit at t1
	// t1 also comes before t2
	if( t1 >= 0 && t1 <= 1)
	{
		result = l * t1 + l1;
		return true;
	}

	// It is a hit at t2
	if( t2 >= 0 && t2 <= 1)
	{
		result = l * t2 + l1;
		return true;
	}

	return false;
}


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


/*
 * Tells whether a boundingbox is coliding with the given circle with radius
 */
bool cMap::BoundingBoxCircleCollision( tBoundingBox box, tVector circle, float radius)
{
	// If we clamp the point to the box, we end up with the point nearest to the circle
	tVector pos = box.Clamp( circle);

	// Get the difference and test whether it is inside the radius
	tVector dif = circle - pos;
	return (dif.LengthSquared() < radius*radius);
}
