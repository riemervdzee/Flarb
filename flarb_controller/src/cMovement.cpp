/*
 * Author: Daniel de Valk
 * Class: Movement in the controller
 *
 *
 */

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include "flarb_mapbuilder/MapImage.h"

int cMovement::Create()
{
	
	return 0;
}


//TODO Angle North would be ideal for heading?!
int cMovement::update(flarb_mapbuilder::MapImage msg)
{
	
}


/*
 *	a save zone around the sick scanner given in meters
 * 	returns the count of detected points in this square
 *	demands that zero points being filterd out before this method
 */ 
int cMovement::saveZone(float meterX, float meterY, const flarb_mapbuilder::MapImage &msg)
{
	//calculate points per meter
	float x = msg.imageX / msg.sizeWidth;
	float y = msg.imageY / msg.sizeHeight;
	
	//calculate points around sick scanner
	float safeX = x * meterX;
	float safeY = y * meterY;
	
	//set particles to zero
	int countParticles = 0;	
	
	//forloop around safe zone sick scanner
	for( int x = (msg.cameraX - (int) safeX); x < ((msg.cameraX + (int) safeX) / 8); x++)
	{
		for( int y = (msg.cameraY - (int) safeY); y < (msg.cameraY + (int) safeY); y++)
		{
			countParticles += CheckFreePixel(x,y, &msg);
		}
	}
	return countParticles;
}

/*
 *	give x, y, msg the Pixel and returns
 *	1 == available
 *	0 == none
 */
int cMovement::CheckBlockedPixel( const flarb_mapbuilder::MapImage &msg, int x, int y)
{	
	//count row length
	int bytesRow = msg.imageX / 8;
	//multiply it to the right row
	int posY = y * bytesRow; 
	
	//how far in the x array
	int posX = x / 8;
	//how far in the byte
	int byteX = x % 8;
	
	//value
	return (msg.data[posX + posY] & ( 1 << byteX ));
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

/*
 * Optimized version to check whole lines on the X axis.
 * NOT FINNISHED!
 *
 * Arguments:
 *    int  x, y    Is the start position
 *    int  width   The width of the line to check (note, it advances to the _right_)
 *
 * Returns:
 *    int  amount of blocking pixels
 */
/*int cMovement::CheckBlockedLineX( const flarb_mapbuilder::MapImage &msg, int x, int y, int width)
{
	// Amount of blocked pixels
	int blocked = 0;

	// Get the basic y-offset
	int posY = y * msg.imageX / 8;

	// Get the index of the first and last byte we need to check
	// Check these special cases first, as we don't check the whole byte
	int pos_start = posY + x / 8;
	int pos_end   = posY + (x + width - 1) / 8;

	//
	uint8_t byte_start = msg.data[pos_start]

	// TODO finish it properly
	return width;
}*/

/*
 * Optimized version to check whole lines on the Y axis.
 *
 * Arguments:
 *    int  x, y    Is the start position
 *    int  height  The height of the line to check (note, it advances to the _bottom_)
 *
 * Returns:
 *    int  amount of blocking pixels
 */
int cMovement::CheckBlockedLineY( const flarb_mapbuilder::MapImage &msg, int x, int y, int height)
{
	// Amount of blocked pixels
	int blocked = 0;

	// count row length
	int bytesRow = msg.imageX / 8;
	// multiply it to the right row
	int posY = y * bytesRow;

	// how far in the x array
	int posX = x / 8;
	// how far in the byte
	int byteX = x % 8;

	// Get the first offset, and get the mask we are trying to test
	int offset = posX + posY;
	int mask   = ( 1 << byteX );

	// Go through the whole line
	for(int i = 0; i < height; i++, offset += bytesRow)
		if( msg.data[offset] & mask)
			blocked++;

	// Return result
	return blocked;
}

