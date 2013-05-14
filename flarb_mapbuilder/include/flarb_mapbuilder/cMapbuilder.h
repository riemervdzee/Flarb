#ifndef CLASS_MAPBUILDER_H
#define CLASS_MAPBUILDER_H

#include "flarb_mapbuilder/MapList.h"

#include "flarb_mapbuilder/cPointCloud.h"

// When the obj has one datapoint, this will be the radius
#define CIRCLE_RADIUS_STANDARD 0.02f

// Maximum radius per circle, this is in meters squared
#define CIRCLE_RADIUS_MAX (0.15f * 0.15f)

/*
 * Class which creates a Map.msg from a cPointCloud object
 */
class cMapbuilder
{
public:
	// Builds from pc, put result in msg
	void Build( const cPointCloud &pc, flarb_mapbuilder::MapList &msg);
};

#endif // CLASS_MAPBUILDER_H

