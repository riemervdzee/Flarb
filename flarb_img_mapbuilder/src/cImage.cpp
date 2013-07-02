// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <cstring>
#include <cmath>

#include "ros/ros.h"

#include "flarb_img_mapbuilder/cImage.h"
using namespace std;


/*
 * Constructor and deconstructor, to allocate data for _frame
 */
cImage::cImage() : _data( new char[ IMAGE_SIZE]) {}
cImage::~cImage() { delete _data;};


/*
 * Get the data pointer
 */
const char* cImage::getData() const
{
	return _data;
}


/*
 * "Clears" the image (fill it with 0)
 */
void cImage::ClearImage()
{
	memset( _data, 0, IMAGE_SIZE);
}


/*
 * Adds the framepoints to the image
 */
void cImage::AddFramePoints( const cFrame &frame)
{
	// Define x/y_old coordinates and set first run to true
	int x_old = -1;
	int y_old = -1;
	bool first = true;

	// Go through all points
	for( unsigned int i = 0; i < frame._dataPoints.size(); i++)
	{
		// Convert coordinates from world-state to image-state
		int x = (int)(frame._dataPoints.at( i).x * (IMAGE_WIDTH  / IMAGE_METER));
		int y = (int)(frame._dataPoints.at( i).y * (IMAGE_HEIGHT / IMAGE_METER));

		// Add offset and invert y (we are going from world-state to image-state)
		x += IMAGE_OFFSET_X;
		y += IMAGE_OFFSET_Y;

		// If this ain't the first pixel, draw it. otherwise skip
		if( !first)
		{
			// If the distance between points ain't too much, draw a line otherwise a dot
			if( ((x-x_old)*(x-x_old) + (y-y_old)*(y-y_old)) < IMAGE_LINE_MAX)
				DrawLine( x_old, y_old, x, y);
			else
				DrawPixel( x, y);
		}
		else
			first = false;

		// Set old points
		x_old = x;
		y_old = y;
	}
}


/*
 * Checks if a given pixel-value is in range
 */
inline bool cImage::PixelInRange( unsigned int x, unsigned int y)
{
	return x < IMAGE_WIDTH && y < IMAGE_HEIGHT;
}


/*
 * Draws a pixel
 */
void cImage::DrawPixel( unsigned int x, unsigned int y)
{
	// Check if in range
	if( !PixelInRange( x, y))
		return;

	// Get byte and bit offset
	unsigned int  bit = x % 8;
	unsigned int byte = x / 8 + y * IMAGE_WIDTH / 8;

	// Set bit
	_data[ byte] |= 1 << bit;
}


/*
 * Draws a line between the two points
 */
void cImage::DrawLine( int x0, int y0, int x1, int y1)
{
	// If both points ain't in range, quit
	if( !PixelInRange( x0, y0 ) && !PixelInRange( x1, y1 ))
		return;

	bool steep = abs(y1 - y0) > abs(x1 - x0);

	if ( steep) {
		std::swap( x0, y0);
		std::swap( x1, y1);
	}

	if ( x0 > x1) {
		std::swap( x0, x1);
		std::swap( y0, y1);
	}

	int deltax = x1 - x0;
	int deltay = abs(y1 - y0);
	int error = deltax / 2;
	int ystep;
	int y = y0;

	if (y0 < y1)
		ystep = +1;
	else
		ystep = -1;

	for ( int x = x0; x < x1; x++)
	{
		if ( steep)
			DrawPixel( (unsigned int) y, (unsigned int) x);
		else
			DrawPixel( (unsigned int) x, (unsigned int) y);

		error = error - deltay;

		if (error < 0) {
			y += ystep;
			error += deltax;
		}
	}
}

