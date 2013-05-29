/*
 * tBoundingBox.cpp -- A standard axis-alligned bounding-box implementation
 */

#include "flarb_simulation/types/tBoundingBox.h"

/*
 * Constructs a boundingBox formed by two vectors. 
 * Note these may be unsorted. min/max x/y are gained from these two vecs
 */
tBoundingBox::tBoundingBox( tVector v1, tVector v2)
{
	// Set min/max X
	if(v1.getX() < v2.getX())
	{
		_min.setX( v1.getX());
		_max.setX( v2.getX());
	}
	else
	{
		_min.setX( v2.getX());
		_max.setX( v1.getX());
	}

	// Ditto, but for Y
	if(v1.getY() < v2.getY())
	{
		_min.setY( v1.getY());
		_max.setY( v2.getY());
	}
	else
	{
		_min.setY( v2.getY());
		_max.setY( v1.getY());
	}
}


/*
 * Returns a vector clamped by this BoundingBox
 */
tVector tBoundingBox::Clamp( const tVector &vec)
{
	float x = vec.getX();
	float y = vec.getY();
	x = (x > _max.getX()) ? _max.getX() : (x < _min.getX()) ? _min.getX() : x;
	y = (y > _max.getY()) ? _max.getY() : (y < _min.getY()) ? _min.getY() : y;

	return tVector( x, y);
}


/*
 * Test whether a vector is inside this boundingbox or not
 */
bool tBoundingBox::Test( const tVector &vec)
{
	return (vec.getX() >= _min.getX() &&
			vec.getX() <= _max.getX() &&
			vec.getY() >= _min.getY() &&
			vec.getY() <= _max.getY());
}

