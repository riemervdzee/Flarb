/*
 * tMatrix.cpp -- Standard matrix class
 */

#include "flarb_controller/types/tMatrix.h"


// Sets the matrix with an angle straight away
tMatrix::tMatrix( const float angle_rad)
{
	setAngle( angle_rad);
}


// Sets the angle (in radians)
void tMatrix::setAngle( const float angle_rad)
{
	c = cos( angle_rad);
	s = sin( angle_rad);
}

// Returns the angle (in radians)
float tMatrix::getAngle() const
{
	return acos( c);
}

// "Normalize" the matrix
void tMatrix::Normalize()
{
	float L = 1.0f / (sqrt(c*c + s*s));
	c *= L;
	s *= L;
}

// Returns a rotated vector with the given length, 0 rad = vector(length, 0)
tVector tMatrix::getVector( const float length) const
{
	return tVector( c * length, s * length);
}

// Returns a rotated vector A, according the matrix
tVector tMatrix::Rotate ( const tVector &A) const
{
	float _x = A.getX() * c + A.getY() *-s;
	float _y = A.getX() * s + A.getY() * c;

	return tVector( _x, _y);
}

// Multiply this matrix with another and return the result
tMatrix  tMatrix::operator * ( const tMatrix &A) const
{
	float _c = c * A.c + s *-A.s;
	float _s = c * A.s + s * A.c;
	return tMatrix( _c, _s);
}

// Multiply this matrix with another and put the result in this matrix
tMatrix &tMatrix::operator *= ( const tMatrix &A)
{
	float _c = c * A.c + s *-A.s;
	float _s = c * A.s + s * A.c;
	c = _c;
	s = _s;
	return *this;
}
