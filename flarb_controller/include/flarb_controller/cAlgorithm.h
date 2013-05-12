#ifndef CLASS_ALGORITHM_H
#define CLASS_ALGORITHM_H

#include "flarb_controller/types/tVector.h"


/*
 * Algorithm class
 */
class cAlgorithm
{
public:
	//
	static bool IntersectCircle( tVector lineStart, tVector lineEnd, 
		tVector circle, float radius, tVector &circleWhenHit);

	//
	static void GetOuterPoints( tVector lineStart, tVector circle, 
		float radius, tVector &p1, tVector &p2);
};

#endif // CLASS_ALGORITHM_H
