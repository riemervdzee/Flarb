#ifndef CLASS_IMAGE_H
#define CLASS_IMAGE_H

#include "ros/ros.h"
#include "flarb_mapbuilder/cFrame.h"


// Image properties
#define IMAGE_WIDTH     1024  // always a multiple of 8!
#define IMAGE_HEIGHT    1024
#define IMAGE_SIZE      (IMAGE_WIDTH * IMAGE_HEIGHT / 8)

#define IMAGE_METER     5.0f  // The amount meters the image represents on each axis
#define IMAGE_OFFSET_X  (IMAGE_WIDTH / 2)      // Camera position X
#define IMAGE_OFFSET_Y  (IMAGE_HEIGHT / 4 * 1) // Y

// The maximum distance in pixels for each line to be drawn
// Note this is squared distance
#define IMAGE_LINE_MAX  (20 * 20)


// Check if FRAME_WIDTH is actually a multiple of 8..
#if (IMAGE_WIDTH != (IMAGE_WIDTH / 8 * 8))
	#error "IMAGE_WIDTH is not a multiple of 8!"
#endif


/*
 * Translates a frame to an image, used for navigation
 */
class cImage
{
public:
	// Constructor and deconstructor, to allocate data for _data
	cImage();
	~cImage();

	// "Clears" the image (fill it with 0)
	void ClearImage();

	// Adds the framepoints to the image
	void AddFramePoints( const cFrame &frame);

	// Get the data pointer
	const char* getData() const;

private:
	// Forbid copy constructor
	cImage( const cImage&);

	// Our frame-data from the laser, but converted to an image
	char *_data;

	// Primitive drawing functions
	bool PixelInRange( int x, int y);
	void DrawPixel( int x, int y);
	void DrawLine( int x0, int y0, int x1, int y1);
};

#endif // CLASS_IMAGE_H

