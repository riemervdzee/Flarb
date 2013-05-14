/*
 * tVector.cpp -- Standard 2D vector class
 */

#include "flarb_controller/types/tVector.h"


// Set the vector to a certain length
tVector& tVector::setLength( float len)
{
	// Limit to only one devision
	float L = len / Length();

	x *= L;
	y *= L;

	return *this;
}

// Normalize
tVector& tVector::Normalize()
{
	// Limit to only one devision
	float L = 1.0f / Length();

	x *= L;
	y *= L;

	return *this;
}

// Makes this vector perpendicular to the existing values (rotating 90 degrees right-hand wise)
tVector& tVector::Perpendicular()
{
	// new vector = Perp( x, y) = (-y, x)
	float temp = x;
	x = -y;
	y = temp;

	return *this;
}

// Clamps the vector to the rectangle formed by min/max
tVector& tVector::Clamp( const tVector& min, const tVector& max)
{
	// Clamp both the X and Y
	x = (x > max.x) ? max.x : (x < min.x) ? min.x : x;
	y = (y > max.y) ? max.y : (y < min.y) ? min.y : y;

	return *this;
}
