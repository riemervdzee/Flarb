#ifndef CLASS_MAP_H
#define CLASS_MAP_H

#include <list>
#include <vector>
#include "flarb_controller/types/tVector.h"
#include "flarb_mapbuilder/MapList.h"

/*
 * Map class
 */
class cMap
{
public:
	//
	void setMapList( const flarb_mapbuilder::MapList &map);

	//
	float FindFreePath( const tVector &input, tVector &output);

	//
	bool IntersectCircle( tVector lineStart, tVector lineEnd, 
		tVector circle, float radius, tVector &circleWhenHit);

	//
	void GetOuterPoints( tVector lineStart, tVector circle, 
		float radius, tVector &p1, tVector &p2);

private:
	std::list<tVector> _WaypointAttempts;
	std::vector<int> _ObjectsCollided;

	const flarb_mapbuilder::MapList *_map;
};

#endif // CLASS_MAP_H
