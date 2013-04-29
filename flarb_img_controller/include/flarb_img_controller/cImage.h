#ifndef CLASS_IMAGE_H
#define CLASS_IMAGE_H

#include "ros/ros.h"
#include "flarb_img_mapbuilder/MapImage.h"

class cImage
{
private:
	// Pointer to the msg _NOTE cImage IS NOT THE OWNER OF THIS OBJ_
	const flarb_img_mapbuilder::MapImage *_msg;

	// Cached values
	int _bytesRow;
	float _Meters2Pixels;
	float _Pixels2Meters;

public:
	// C-tor
	cImage( const flarb_img_mapbuilder::MapImage *msg) :
		_msg( msg),
		_bytesRow( _msg->imageX / 8),
		_Meters2Pixels( _msg->imageX / _msg->sizeWidth),
		_Pixels2Meters( _msg->sizeWidth / _msg->imageX)
	{}

	// Count the amount of pixels, supporting a few basic geo functions
	int CountBlockedPixel     ( int x, int y) const;
	int CountBlockedLineX     ( int x, int y, int width) const;
	int CountBlockedLineY     ( int x, int y, int height) const;
	int CountBlockedRectangle ( int x, int y, int width, int height) const;

	// Advances from the position given to the left/right, till the first bit is found
	int GetXLeft  (int x, int y) const;
	int GetXRight (int x, int y) const;

	// Getters
	const flarb_img_mapbuilder::MapImage* getMapImage() const { return _msg;}
	int   getBytesRow() const { return _bytesRow;}
	float getMeters2Pixels() const {return _Meters2Pixels;}
	float getPixels2Meters() const {return _Pixels2Meters;}
};

#endif // CLASS_IMAGE_H
