#ifndef CLASS_IMAGE_H
#define CLASS_IMAGE_H

#include "ros/ros.h"


#define IMAGE_WIDTH     2056  // always a multiple of 8!
#define IMAGE_HEIGHT    2056
#define IMAGE_SIZE      IMAGE_WIDTH * IMAGE_HEIGHT / 8

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
	// Constructor and deconstructor, to allocate data for _frame
	cImage();
	~cImage();
	
	// Our frame-data from the laser, but converted to an image
	// Note this data is in public domain, so it can be used
	char *_frame;

private:
	// Forbid copy constructor
	cImage( const cMapImage&);
};

#endif // CLASS_IMAGE_H

