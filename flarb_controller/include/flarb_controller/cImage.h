#ifndef CLASS_IMAGE_H
#define CLASS_IMAGE_H

#include "ros/ros.h"
#include "flarb_mapbuilder/MapImage.h"

class cImage
{
public:
	//ctor
	cImage( const flarb_mapbuilder::MapImage *msg);

	// Count the amount of pixels, supporting a few basic geo functions
	int CountBlockedPixel     ( int x, int y) const;
	int CountBlockedLineX     ( int x, int y, int width) const;
	int CountBlockedLineY     ( int x, int y, int height) const;
	int CountBlockedRectangle ( int x, int y, int width, int height) const;

	// Advances from the position given to the left/right, till the first bit is found
	int GetXLeft  (int x, int y) const;
	int GetXRight (int x, int y) const;

private:
	// Pointer to the msg _NOTE cImage IS NOT THE OWNER OF THIS OBJ_
	const flarb_mapbuilder::MapImage *_msg;

	// Cached values
	int _bytesRow;
};

#endif // CLASS_IMAGE_H
