#include <cmath>

#include "flarb_controller/cAlgorithm.h"

// Based on http://mathworld.wolfram.com/Circle-LineIntersection.html
// http://stackoverflow.com/questions/7060519/
// TODO cleanup
bool cAlgorithm::IntersectCircle( tVector lineStart, tVector lineEnd, 
		tVector circle, float radius, tVector &circleWhenHit)
{
	circleWhenHit = tVector();

	// find the closest point on the line segment to the center of the circle
	tVector line = lineEnd - lineStart;
	float lineLength = line.Length();
	tVector lineNorm = (1/lineLength)*line;
	tVector segmentToCircle = circle - lineStart;
	float closestPointOnSegment = segmentToCircle.Dot(line) / lineLength;

	// Special cases where the closest point happens to be the end points
	tVector closest;
	if (closestPointOnSegment < 0) closest = lineStart;
	else if (closestPointOnSegment > lineLength) closest = lineEnd;
	else closest = lineStart + closestPointOnSegment*lineNorm;

	// Find that distance.  If it is less than the radius, then we 
	// are within the circle
	tVector distanceFromClosest = circle - closest;
	float distanceFromClosestLength = distanceFromClosest.Length();
	if (distanceFromClosestLength > radius) return false;

	// So find the distance that places the intersection point right at 
	// the radius.  This is the center of the circle at the time of collision
	// and is different than the result from Doswa
	tVector offset = (radius - distanceFromClosestLength) *
					 ((1/distanceFromClosestLength)*distanceFromClosest);
	circleWhenHit = circle - offset;

	return true;
}

void cAlgorithm::GetOuterPoints( tVector lineStart, tVector circle, 
	float radius, tVector &p1, tVector &p2)
{
	tVector startToCircle = circle - lineStart;
	float ltot = startToCircle.Length();
	tVector N = (1/ltot)*startToCircle;
	tVector P = N;
	N.Perpendicular();

	float r2 = radius*radius;
	float l1 = ltot - (r2 / ltot);
	float h  = (sqrt(ltot*ltot-r2) * radius) / ltot;

	tVector l1Vec = l1 * N;
	tVector hVec = h * P;
	p1 = l1Vec + hVec;
	p2 = l1Vec - hVec;
}
