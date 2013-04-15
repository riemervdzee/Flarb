#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include "flarb_mapbuilder/MapImage.h"

#include "flarb_controller/cImage.h"
using namespace std;

// Prototypes
static unsigned int CountBitsSet( unsigned int v);

/*
 *  Constructor
 */
cImage::cImage( const flarb_mapbuilder::MapImage *msg) : _msg(msg)
{
	_bytesRow = _msg->imageX / 8;
}


/*
 *  give x, y, msg the Pixel and returns
 *  1 == available
 *  0 == none
 */
int cImage::CountBlockedPixel( int x, int y)
{
	//multiply it to the right row
	int posY = y * _bytesRow;
	
	//how far in the x array
	int posX = x / 8;

	//how far in the byte
	int byteX = x % 8;
	
	//value
	if(_msg->data[posX + posY] & ( 1 << byteX ))
		return 1;
	else
		return 0;
}


/*
 * Optimized version to check whole lines on the X axis.
 * If you are about to check a rectangle, this version is faster than the Y version
 *
 * This is quite a mess, I know I know..
 *
 * Arguments:
 *    int  x, y    Is the start position
 *    int  width   The width of the line to check (note, it advances to the _right_)
 *
 * Returns:
 *    int  amount of blocking pixels
 */
int cImage::CountBlockedLineX( int x, int y, int width)
{
// Amount of blocked pixels
	int blocked = 0;

	// Get the basic y-offset
	int posY = y * _bytesRow;

	// Get the index of the first and last byte we need to check
	// Check these special cases first, as we don't check the whole byte
	int pos_start = posY + x / 8;
	int pos_end   = posY + (x + width - 1) / 8;

	// Bits we are interested in for the special cases
	// Basically we shift the amount of bits we don't want
	int bit_start = x % 8;

	// Invert the bit (8 - bit_end), but add 1 again.
	int bit_end   =  7 - ((x + width - 1) % 8);

	// Get the special cases
	uint8_t temp;
	temp = _msg->data[pos_start] >> bit_start;
	blocked += CountBitsSet( temp);


	// Increase start-position and get the pos_width
	pos_start++;
	int pos_width = pos_end - pos_start;

	// Go through all remaining bytes
	for(int i = 0; i < pos_width; i++, pos_start++)
	{
		temp = _msg->data[pos_start];
		blocked += CountBitsSet( temp);
	}

	// Get the last, pos_end for caching
	temp = _msg->data[pos_end] << bit_end;
	blocked += CountBitsSet( temp);

	// Return blocked
	return blocked;
}


/*
 * Optimized version to check whole lines on the Y axis.
 * Note this is quite slow, due we have to jump through the whole array
 *
 * Arguments:
 *    int  x, y    Is the start position
 *    int  height  The height of the line to check (note, it advances to the _bottom_)
 *
 * Returns:
 *    int  amount of blocking pixels
 */
int cImage::CountBlockedLineY( int x, int y, int height)
{
	// Amount of blocked pixels
	int blocked = 0;

	// multiply bytesrow with y, to get the y-offset
	int posY = y * _bytesRow;

	// how far in the x array
	int posX = x / 8;
	// how far in the byte
	int byteX = x % 8;

	// Get the first offset, and get the mask we are trying to test
	int offset = posX + posY;
	int mask   = ( 1 << byteX );

	// Go through the whole line
	for(int i = 0; i < height; i++, offset += _bytesRow)
		if( _msg->data[offset] & mask)
			blocked++;

	// Return result
	return blocked;
}


/*
 * Helper func to count a whole Rectangle. Uses CountBlockedLineX
 *
 * Arguments:
 *    int  x, y    Is the start position
 *    int  width   The width of the line to check (note, it advances to the _right_)
 *    int  height  The height of the line to check (note, it advances to the _bottom_)
 *
 * Returns:
 *    int  amount of blocking pixels
 */
int cImage::CountBlockedRectangle ( int x, int y, int width, int height)
{
	int amount = 0;

	// All line X
	for(int _y = 0; _y < height; _y++)
	{
		amount += CountBlockedLineX( x, y + _y, width);
	}

	return amount;
}


/*
 * Static helper function to count the amount of bits set in a uint
 * From: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetKernighan
 */
static unsigned int CountBitsSet( unsigned int v)
{
	unsigned int c;
	v = v - ((v >> 1) & 0x55555555);                    // reuse input as temporary
	v = (v & 0x33333333) + ((v >> 2) & 0x33333333);     // temp
	c = (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24; // count

	return c;
}

