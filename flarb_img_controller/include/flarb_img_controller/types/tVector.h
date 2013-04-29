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
	inline tVector(): x(0), y(0){}
	inline tVector( float _x, float _y): x(_x), y(_y){}

	// Get commands
	inline float getX() const {return x;}
	inline float getY() const {return y;}

	// Set commands
	inline tVector& setX( float _x) {x = _x; return *this;}
	inline tVector& setY( float _y) {y = _y; return *this;}

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
	friend tVector operator * (float s, const tVector& A) { return tVector(A.x*s, A.y*s); }

	// Cross product
	inline float Cross (const tVector &A) const { return (x * A.y) - (y * A.x); }

	// Dot product
	inline float Dot (const tVector &A) const { return (x*A.x) + (y*A.y); }

	// Return length
	inline float Length(void) const {return (float) sqrt(x*x + y*y); }

	// Return squared length
	inline float LengthSquared(void) const {return x*x + y*y; }

	// Normalize
	tVector& setLength( float len)
	{
		// Limit to only one devision
		float L = len / Length();

		// Multiply X and Y with L
		x *= L;
		y *= L;

		// Return ourself
		return *this;
	}

	// Normalize
	tVector& Normalize(void)
	{
		// Limit to only one devision
		float L = 1.0f / Length();

		// Multiply X and Y with L
		x *= L;
		y *= L;

		// Return ourself
		return *this;
	}

	// Makes this vector perpendicular to the existing one (rotating 90 degrees right-hand wise)
	tVector& Perpendicular(void)
	{
		float temp = x;
		x = -y;
		y = temp;

		//Return ourself
		return *this;
	}

	//Clamp vector to rectangle formed by min/max
	tVector& Clamp(const tVector& min, const tVector& max)
	{
		// Clamp X
		x = (x > max.x) ? max.x : (x < min.x) ? min.x : x;

		// Clamp Y
		y = (y > max.y) ? max.y : (y < min.y) ? min.y : y;

		//Return ourself
		return *this;
	}
};


#endif // FLARB_VECTOR_H_

