/*
 * tMatrix.h -- Standard matrix class
 */

#ifndef FLARB_MATRIX_H_
#define FLARB_MATRIX_H_

#include <cmath>
#include <cstdio>
#include "flarb_controller/types/tVector.h"


class tMatrix
{
private:
	// The variables representing the matrix. They are the sinus & cosinus results of the angle
	float c, s;

public:
	// Constructors
	inline tMatrix(): c(1), s(0){}
	inline tMatrix( float _c, float _s): c(_c), s(_s){}

	// Get functions
	inline float getC() { return c;}
	inline float getS() { return s;}

	// Sets the angle
	void setAngle( const float angle_rad)
	{
		c = cos( angle_rad);
		s = sin( angle_rad);
	}

	// Returns the angle (in radians)
	float getAngle() const
	{
		return acos(c);
	}

	// Normalize the matrix
	void Normalize()
	{
		float L = 1.0f / (sqrt(c*c + s*s));
		c *= L;
		s *= L;
	}

	// Print DEBUG info
	inline void Print() const
	{
		printf("xX: %f xY: %f\n", c, s);
	}

	// Rotates a vector and returns it
	tVector Rotate ( tVector &A) const
	{
		float _x = A.getX() * c + A.getY() *-s;
		float _y = A.getX() * s + A.getY() * c;

		return tVector( _x, _y);
	}

	// Multiply this matrix with another and return the result
	tMatrix  operator * ( const tMatrix &A) const
	{
		float _c = c * A.c + s *-A.s;
		float _s = c * A.s + s * A.c;
		return tMatrix( _c, _s);
	}

	// Multiply this matrix with another and put the result in this matrix
	tMatrix &operator *= ( const tMatrix &A)
	{
		float _c = c * A.c + s *-A.s;
		float _s = c * A.s + s * A.c;
		c = _c;
		s = _s;
		return *this;
	}
};

#endif /* R_MATRIX_H_ */

