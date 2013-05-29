// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_simulation/config.h"
#include "flarb_simulation/cController.h"
#include "flarb_simulation/Graphics.h"
using namespace std;

// Generates a map
bool cMap::Create()
{
	// Construct plant
	float xf = 0.0f;
	for( int x = 0; x < MAP_ROWS; x++, xf += MAP_ROW_OFFSET)
	{
		float yf = 0.0f;
		for(int y = 0; y < MAP_ROW_PLANTS; y++, yf += MAP_PLANT_OFFSET)
		{
			Plant s;
			s.x = xf;
			s.y = yf;

			_plants.push_back( s);
		}
	}
	
	return true;
}

// Executed when the Node is exiting
void cMap::Destroy()
{

}

// Draws the map obj
void cMap::Draw() const
{
	for(unsigned int i = 0; i < _plants.size(); i++)
	{
		const Plant s = _plants[i];
		drawCircle( s.x, s.y, MAP_PLANT_WIDTH, 8);
	}
}


float cMap::TestRayDistance( tVector l1, float angle) const
{
	tVector l2 = tVector( (l1.getX() + cos( angle)), (l1.getY() + sin( angle)));

	float result = 22.0f;
	for(unsigned int i = 0; i < _plants.size(); i++)
	{
		const Plant s = _plants[i];
		float res;
		if( IntersectCircle( l1, l2, tVector(s.x, s.y), MAP_PLANT_WIDTH, res))
		{
			if (res < result)
				result = res;
		}
	}

	return result;
}


/*
 * Tells us whether a INfinite line (l1, l2) intersects with the given circle and
 * radius. Puts the distance in result
 */
bool cMap::IntersectCircle( tVector l1, tVector l2, tVector circle,
	float radius, float &result) const
{
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

	result = t1;
	return true;
}
