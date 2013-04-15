#ifndef CLASS_IMAGE_H
#define CLASS_IMAGE_H

#include "ros/ros.h"
#include "flarb_mapbuilder/MapImage.h"

class cImage
{
public:
	//ctor
	cImage( const flarb_mapbuilder::MapImage *msg);

	int CountBlockedPixel     ( int x, int y);
	int CountBlockedLineX     ( int x, int y, int width);
	int CountBlockedLineY     ( int x, int y, int height);
	int CountBlockedRectangle ( int x, int y, int width, int height);

private:
	const flarb_mapbuilder::MapImage *_msg;
	
	int _bytesRow;
};

#endif // CLASS_IMAGE_H
