/*
 * tBoundingBox.h -- A standard axis-alligned bounding-box implementation
 */


#ifndef FLARB_BOUNDINGBOX_H_
#define FLARB_BOUNDINGBOX_H_

#include "flarb_controller/types/tVector.h"

class tBoundingBox
{
private:
	// Variables representing the vector
	tVector _min;
	tVector _max;

public:
	/*
	 * Empty constructor doing nothing (the vectors are set to 0 anyway)
	 */
	inline tBoundingBox() {}
	
	/*
	 * Constructs a boundingBox formed by two vectors. 
	 * Note these may be unsorted. min.x and etc. are gained from these two vecs
	 */
	tBoundingBox( tVector v1, tVector v2);

	/*
	 * Getter commands
	 */
	inline tVector getMin() const { return _min;}
	inline tVector getMax() const { return _max;}

	/*
	 * Setters commands, NOTE is no guarantee that min < max after these funcs
	 */
	inline tBoundingBox& setMin( tVector v) { _min = v; return *this;}
	inline tBoundingBox& setMax( tVector v) { _max = v; return *this;}


	/*
	 * Returns a vector clamped by this BoundingBox
	 */
	tVector Clamp( const tVector &vec);


	/*
	 * Test whether a vector is inside this boundingbox or not
	 */
	bool Test( const tVector &vec);
};


#endif // FLARB_BOUNDINGBOX_H_

