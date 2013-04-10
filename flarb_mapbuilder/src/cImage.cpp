// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <cstring>

#include "ros/ros.h"

#include "flarb_mapbuilder/cImage.h"
using namespace std;

// Constructor and deconstructor, to allocate data for _frame
cImage::cImage() : _data( new char[ IMAGE_SIZE]), _x( 0), _y( 0) {}
cImage::~cImage() { delete _data;};

void cImage::ClearImage()
{
	memset( _data, 0, IMAGE_SIZE);
}

/*void cImage::SetTransformation( float x, float y TODO rotation)
{
	
}*/

void cImage::AddFramePoints( const cFrame &frame)
{
	// Go through all points
	for( unsigned int i = 0; i < frame._dataPoints.size(); i++)
	{
		// Convert coordinates from world-state to image-state
		int x = (int)(frame._dataPoints.at( i).x * ((float)IMAGE_WIDTH  / IMAGE_METER));
		int y = (int)(frame._dataPoints.at( i).y * ((float)IMAGE_HEIGHT / IMAGE_METER));

		// Add offset
		x += IMAGE_OFFSET_X;
		y += IMAGE_OFFSET_Y;

		// Invert Y, as we are going from Y-up to Y-down (for images)
		y = IMAGE_HEIGHT - y;

		// Check if in range
		if( x >= 0 && x < IMAGE_WIDTH && y >= 0 && y < IMAGE_HEIGHT)
		{
			// Get byte and bit offset
			int  bit = x % 8;
			int byte = x / 8 + y * IMAGE_WIDTH / 8;

			// Set bit
			_data[ byte] |= 1 << bit;
		}
	}
}

