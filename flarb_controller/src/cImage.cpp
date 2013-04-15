#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include "flarb_mapbuilder/MapImage.h"

#include "flarb_controller/cImage.h"

static unsigned int CountBitsSet ( uint8_t val);

cImage::cImage(const flarb_mapbuilder::MapImage *msg) : _msg(msg)
{
	_bytesRow = _msg->imageX / 8;
}


int cImage::CountBlockedPixel( int x, int y)
{
	//multiply it to the right row
	int posY = y * _bytesRow;
	
	//how far in the x array
	int posX = x / 8;

	//how far in the byte
	int byteX = x % 8;
	
	//value
	return (_msg->data[posX + posY] & ( 1 << byteX ));
}

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
	int pos_width = pos_end - pos_start - 2;
	// -2 = -1 normally, and -1 as we already parse pos_end at the end

	// Go through all remaining bytes
	for(int i = 0; i < pos_width; i++, pos_start++)
	{
		temp = _msg->data[pos_start];
		blocked += CountBitsSet( temp);
	}

	// Get the last, pos_end for caching
	temp = _msg->data[pos_end] << bit_end;
	blocked += CountBitsSet( temp);

	// TODO pos_start == pos_end by now, test it!
	// Return blocked
	return blocked;
}

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
 * Static helper function to count the amount of bits set in a uint
 * From: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetKernighan
 */
static unsigned int CountBitsSet ( uint8_t val)
{
	// c accumulates the total bits set in v
	unsigned int count;

	// The nice part of this method is, it only itterates for each bit set
	for (count = 0; val; count++)
	{
		val &= val - 1; // clear the least significant bit set
	}

	return count;
}
