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
	inline tMatrix(): c( 1), s( 0){}
	inline tMatrix( float _c, float _s): c( _c), s( _s){}
	tMatrix( const float angle_rad);

	// Get functions
	inline float getC() { return c;}
	inline float getS() { return s;}
	
	// Print DEBUG info
	inline void Print() const
	{
		printf("xX: %f xY: %f\n", c, s);
	}


	// Sets the angle (in radians)
	void setAngle( const float angle_rad);

	// Returns the angle (in radians)
	float getAngle() const;

	// "Normalize" the matrix
	void Normalize();

	// Returns a rotated vector with the given length, 0 rad = vector(length, 0)
	tVector getVector( const float length) const;

	// Returns a rotated vector A, according the matrix
	tVector Rotate ( const tVector &A) const;

	// Multiply this matrix with another and return the result
	tMatrix  operator * ( const tMatrix &A) const;

	// Multiply this matrix with another and put the result in this matrix
	tMatrix &operator *= ( const tMatrix &A);
};

#endif /* R_MATRIX_H_ */

