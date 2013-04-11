// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <cstring>
#include <cmath>

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

		// Add offset
		x += IMAGE_OFFSET_X;
		y += IMAGE_OFFSET_Y;

		// Invert Y, as we are going from Y-up to Y-down (for images)
		y = IMAGE_HEIGHT - y;

		// If this ain't the first pixel, draw it. otherwise skip
		if( !first)
			DrawLine( x, y, x_old, y_old);
		else
			first = false;

		// Set old points
		x_old = x;
		y_old = y;
	}
}

inline bool cImage::PixelInRange( int x, int y)
{
	return x >= 0 && x < IMAGE_WIDTH && y >= 0 && y < IMAGE_HEIGHT;
}

void cImage::DrawPixel( int x, int y)
{
	// Check if in range
	if( !PixelInRange( x, y))
		return;

	// Get byte and bit offset
	int  bit = x % 8;
	int byte = x / 8 + y * IMAGE_WIDTH / 8;

	// Set bit
	_data[ byte] |= 1 << bit;
}

/*
 * TODO performance, integrate DrawPixel into this func. to avoid recalculation
 */
void cImage::DrawLine( int x0, int y0, int x1, int y1)
{
	// Check if in range
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
			DrawPixel( y, x);
		else
			DrawPixel( x, y);

		error = error - deltay;

		if (error < 0) {
			y = y + ystep;
			error = error + deltax;
		}
	}
}

