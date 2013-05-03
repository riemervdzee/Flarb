#include "flarb_mapbuilder/cMapbuilder.h"

void cMapbuilder::Build( const cPointCloud& pc, flarb_mapbuilder::Map& msg)
{
	const std::vector<tVector> points  = pc.getVectorPoints();
	const std::vector<sObject> objects = pc.getVectorObjects();

	// Go through all PC-objects
	for( unsigned int i = 0; i < objects.size(); i++)
	{
		sObject obj = objects[i];

		// Special case: when the PC-obj has only one datapoint
		if( obj.length == 1)
		{
			flarb_mapbuilder::Object cobj;
			cobj.id     = i;
			tVector p   = points[ obj.index];
			cobj.x      = p.getX();
			cobj.y      = p.getY();
			cobj.radius = CIRCLE_RADIUS_STANDARD;
			msg.list.push_back( cobj);

			continue;
		}

		// We create circles which cover all of the points of the obj, with a 
		// given maxradius. As we don't want to check every point with eachother
		// we assume the points are "sorted" going left to right, or visa versa. 
		// This way we only have to check with the first vector in the circle
		tVector pFirst = points[ obj.index];
		tVector pSum   = pFirst;  // Note, this becomes the circle position
		int count = 1;

		// Go through the remaining datapoints
		for( unsigned int j = 1; j < obj.length; j++)
		{
			tVector p = points[ obj.index + j];

			// Check if we can combine
			if( (p - pFirst).LengthSquared() < CIRCLE_RADIUS_MAX)
			{
				pSum += p;
				count++;
			}
			else
			{
				// Get average TODO maybe this ain't really neat
				pSum /= count;

				// Construct the obj-circle for the previous points
				flarb_mapbuilder::Object cobj;
				cobj.id     = i;
				cobj.x      = pSum.getX();
				cobj.y      = pSum.getY();

				// Depending on the count, set radius then push the result
				if( count == 1)
					cobj.radius = CIRCLE_RADIUS_STANDARD;
				else
					cobj.radius = (pSum - pFirst).Length();

				msg.list.push_back( cobj);

				// Init for the next obj,
				pFirst = pSum = p;
				count = 1;
			}
		}
		
		// Get average TODO maybe this ain't really neat
		pSum /= count;

		// Construct the last obj
		flarb_mapbuilder::Object cobj;
		cobj.id     = i;
		cobj.x      = pSum.getX();
		cobj.y      = pSum.getY();

		if( count == 1)
			cobj.radius = CIRCLE_RADIUS_STANDARD;
		else
			cobj.radius = (pSum - pFirst).Length();

		msg.list.push_back( cobj);
	}
}
