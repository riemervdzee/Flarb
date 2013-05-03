/*
 * tVector.h -- Standard 2D vector class
 */


#ifndef FLARB_VECTOR_H_
#define FLARB_VECTOR_H_

#include <math.h>
#include <stdio.h>

class tVector
{
private:
	// Variables representing the vector
	float x, y;

public:
	// Constructors
	inline tVector(): x( 0), y( 0) {}
	inline tVector( float _x, float _y): x( _x), y( _y) {}

	// Get commands
	inline float getX() const { return x;}
	inline float getY() const { return y;}

	// Set commands
	inline tVector& setX( float _x) { x = _x; return *this;}
	inline tVector& setY( float _y) { y = _y; return *this;}


	// Vector adding/subtracting
	inline tVector  operator  + (const tVector &A) const {return tVector( x + A.x, y + A.y);}
	inline tVector  operator  - (const tVector &A) const {return tVector( x - A.x, y - A.y);}
	inline tVector  operator  * (const float s)    const {return tVector( x * s, y * s);}
	inline tVector  operator  / (const float s)    const {return tVector( x / s, y / s);}

	// Scalar multiplication/dividing
	inline tVector& operator += (const tVector &A) {x += A.x; y += A.y; return *this;}
	inline tVector& operator -= (const tVector &A) {x -= A.x; y -= A.y; return *this;}
	inline tVector& operator *= (const float s)    {x *= s; y *= s; return *this;}
	inline tVector& operator /= (const float s)    {x /= s; y /= s; return *this;}

	// Changes the sign of a vector
	inline tVector  operator -(void) const {return tVector(-x, -y); }

	// Multiplies with a scalar, so we can write s * tVector where s is the scalar
	inline friend tVector operator * (float s, const tVector& A) { return tVector(A.x*s, A.y*s); }

	// Cross product
	inline float Cross (const tVector &A) const { return (x * A.y) - (y * A.x); }

	// Dot product
	inline float Dot (const tVector &A) const { return (x*A.x) + (y*A.y); }

	// Return length
	inline float Length() const {return sqrt(x*x + y*y); }

	// Return squared length
	inline float LengthSquared() const {return x*x + y*y; }

	// Set the vector to a certain length
	tVector& setLength( float len);

	// Normalize the vector
	tVector& Normalize();

	// Makes this vector perpendicular to the existing values (rotating 90 degrees right-hand wise)
	tVector& Perpendicular();

	// Clamps the vector to the rectangle formed by min/max
	tVector& Clamp( const tVector& min, const tVector& max);
};


#endif // FLARB_VECTOR_H_

