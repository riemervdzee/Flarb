// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"

#include "flarb_simulation/config.h"
#include "flarb_simulation/cController.h"
#include "flarb_simulation/Graphics.h"
using namespace std;

// Called when started
bool cMap::Create()
{
	srand (time(NULL));
	return true;
}

// Executed when the Node is exiting
void cMap::Destroy()
{

}

// Adds a single plant, note it adds a small random val
void cMap::Add( std::string str)
{
	stringstream s;
	Plant p;

	s << str;
	s >> p.x;
	s >> p.y;

	// Adds a (+-2cm max) difference on the pos given
	p.x += ((float)rand() / ((float)RAND_MAX*25) - 0.02f);
	p.y += ((float)rand() / ((float)RAND_MAX*25) - 0.02f);

	_plants.push_back( p);
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

	float result  = 22.0f;
	//float result2 = result*result;
	for(unsigned int i = 0; i < _plants.size(); i++)
	{
		const Plant   s    = _plants[i];
		const tVector sVec = tVector(s.x, s.y);
		float res;
		//if( (sVec - l1).LengthSquared() < result2)
		//{
			if( IntersectCircle( l1, l2, sVec, MAP_PLANT_WIDTH, res))
			{
				if (res < result)
				{
					result  = res;
		//			result2 = result*result;
				}
			}
		//}
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
