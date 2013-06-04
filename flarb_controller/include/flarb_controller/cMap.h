#ifndef CLASS_MAP_H
#define CLASS_MAP_H

#include <list>
#include <vector>
#include "flarb_controller/types/tVector.h"
#include "flarb_controller/types/tBoundingBox.h"
#include "flarb_mapbuilder/MapList.h"

// One of the possible return options of FindFreePath, tells there is no solution
#define FREEPATH_NO_SOLUTION -5.0f


/*
 * Map class
 */
class cMap
{
public:
	/*
	 * Sets the map list to use for the upcoming functions
	 */
	void setMapList( const flarb_mapbuilder::MapList &map);


	/************************
	 Advanced functions
	*************************/

	/*
	 * Checks whether "input" is accessable. The ending result is put in "output"
	 * Returns the dotproduct between input and output. range is 1 to -1
	 * 1 means input = output. 0 = 90 degrees difference, -1 = 180 degrees difference
	 * FREEPATH_NO_SOLUTION = special case, output = {0,0} no solution possible
	 */
	float FindFreePath( const float protection_margin, const tVector &input, tVector &output);

	/*
	 * Tells whether a bbaa region with the map
	 */
	bool CheckIntersectionRegion( const tBoundingBox region);

	/************************
	 Basic functions
	*************************/

	/*
	 * Tells us whether a finite line (l1, l2) intersects with the given circle and
	 * radius. Puts the point of first contact in result
	 * Based on http://mathworld.wolfram.com/Circle-LineIntersection.html
	 */
	bool IntersectCircle( tVector l1, tVector l2, tVector circle, float radius, tVector &result);

	/*
	 * Having a point (lineStart) and a circle (circle + radius), there are two
	 * lines from this point where there is only one intersection with the circle.
	 * This function puts the colisionpoints of these two lines into p1 and p2
	 *
	 * See the documentation for details
	 */
	void GetOuterPoints( tVector lineStart, tVector circle, float radius, tVector &p1, tVector &p2);

	/*
	 * Tells whether a boundingbox is coliding with the given circle with radius
	 */
	bool BoundingBoxCircleCollision( tBoundingBox box, tVector circle, float radius);


private:
	// Used in FindFreePath
	std::list<tVector>        _WaypointAttempts;
	std::vector<unsigned int> _ObjectsCollided;

	// Set by setMapList, used by algorithms
	const flarb_mapbuilder::MapList *_map;
};

#endif // CLASS_MAP_H
