#ifndef CLASS_IMAGE_H
#define CLASS_IMAGE_H

#include "ros/ros.h"

#include "flarb_mapbuilder/cFrame.h"


#define IMAGE_WIDTH     512  // always a multiple of 8!
#define IMAGE_HEIGHT    512
#define IMAGE_SIZE      (IMAGE_WIDTH * IMAGE_HEIGHT / 8)

#define IMAGE_METER     3.0f // 
#define IMAGE_OFFSET_X  (IMAGE_WIDTH / 2)
#define IMAGE_OFFSET_Y  (IMAGE_HEIGHT / 4 * 2)

// Check if FRAME_WIDTH is actually a multiple of 8
#if (IMAGE_WIDTH != (IMAGE_WIDTH / 8 * 8))
	#error "IMAGE_WIDTH is not a multiple of 8!"
#endif

/*
 * 
 */
class cImage
{
public:
	// Our frame-data from the laser, but converted to an image
	// Note this data is in public domain, so it can be used
	char *_data;

	// Constructor and deconstructor, to allocate data for _frame
	cImage();
	~cImage();

	// Clear image
	void ClearImage();

	// 
	//void SetTransformation( float x, float y /*TODO rotation*/);

	// Add the framepoints to the Image_data
	void AddFramePoints( const cFrame &frame);


private:
	// Forbid copy constructor
	cImage( const cImage&);

	//
	bool PixelInRange( int x, int y);
	void DrawPixel( int x, int y);
	void DrawLine( int x0, int y0, int x1, int y1);

	// Transformation
	float _x, _y;
};

#endif // CLASS_IMAGE_H

