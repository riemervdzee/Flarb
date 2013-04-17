#include <iostream>
#include <cstring>
#include "flarb_mapbuilder/MapImage.h"

#include "flarb_controller/cImage.h"
using namespace std;

// TODO add error for non x86-64 arches

// Prototypes of helper functions
static unsigned int CountBitsSet( unsigned int v);
static inline int asm_ffs( int x);
static inline int asm_fls( int x);

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
int cImage::CountBlockedPixel( int x, int y) const
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
int cImage::CountBlockedLineX( int x, int y, int width) const
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
int cImage::CountBlockedLineY( int x, int y, int height) const
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
int cImage::CountBlockedRectangle ( int x, int y, int width, int height) const
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
 * Advances from the position given to the left, till the first bit is found
 *
 * X, Y   the position where to start (note, the current position is also checked)
 *
 * Returns, position of first bit encountered. -1 if no bit is found (highly unlikely)
 */
int cImage::GetXLeft( int x, int y) const
{
	// multiply bytesrow with y, to get the y-offset
	int posY = y * _bytesRow;

	// how far in the x array
	int posX = x / 8;

	// The special case is the byte where X already belongs in
	int bitPosX = 7 - ((x - 1) % 8);

	// Generic offset
	int offset = posX + posY;

	// Mask the bits we don't want to 0.
	uint8_t temp = ( _msg->data[offset] << bitPosX) >> bitPosX;

	// Check if we got a result
	int ret = asm_ffs( temp);
	if(ret != 0)
		return ret + (posX * 8);

	// We handled the first byte, so increase
	posX++;
	offset++;

	// Go through all remaining bytes
	for( ; (unsigned int)posX < _msg->imageX; posX++, offset++)
	{
		int ret = asm_ffs( _msg->data[offset]);
		if(ret != 0)
			return ret + (posX * 8);
	}

	return -1;
}


/*
 * Advances from the position given to the right, till the first bit is found
 */
int cImage::GetXRight( int x, int y) const
{
	// multiply bytesrow with y, to get the y-offset
	int posY = y * _bytesRow;

	// how far in the x array
	int posX = x / 8;

	// The special case is the byte where X already belongs in
	int bitPosX = x % 8;

	// Generic offset
	int offset = posX + posY;

	// Mask the bits we don't want to 0.
	uint8_t temp = ( _msg->data[offset] >> bitPosX) << bitPosX;

	// Check if we got a result
	int ret = asm_ffs( temp);
	if(ret != 0)
		return ret + (posX * 8);

	// We handled the first byte, so decrease
	posX--;
	offset--;

	// Go through all remaining bytes
	for( ; posX >= 0; posX--, offset--)
	{
		int ret = asm_ffs( _msg->data[offset]);
		if(ret != 0)
			return ret + (posX * 8);
	}

	return -1;
}


/*
 * Static helper function to count the amount of bits set in a uint
 * From: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
 */
static unsigned int CountBitsSet( unsigned int v)
{
	unsigned int c;
	v = v - ((v >> 1) & 0x55555555);                    // reuse input as temporary
	v = (v & 0x33333333) + ((v >> 2) & 0x33333333);     // temp
	c = (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24; // count

	return c;
}

/**
 * ffs - find first set bit in word
 * @x: the word to search
 *
 * This is defined the same way as the libc and compiler builtin ffs
 * routines, therefore differs in spirit from the other bitops.
 *
 * ffs(value) returns 0 if value is 0 or the position of the first
 * set bit if value is nonzero. The first (least significant) bit
 * is at position 1.
 * From: https://github.com/torvalds/linux/blob/master/arch/x86/include/asm/bitops.h
 */
static inline int asm_ffs(int val)
{
	int ret;
	asm("bsfl %1,%0"
		: "=r" (ret)
		: "rm" (val), "0" (-1));
	return ret + 1;
}

/**
 * fls - find last set bit in word
 * @x: the word to search
 *
 * This is defined in a similar way as the libc and compiler builtin
 * ffs, but returns the position of the most significant set bit.
 *
 * fls(value) returns 0 if value is 0 or the position of the last
 * set bit if value is nonzero. The last (most significant) bit is
 * at position 32.
 * From: https://github.com/torvalds/linux/blob/master/arch/x86/include/asm/bitops.h
 */
static inline int asm_fls(int val)
{
	int ret;
	asm("bsrl %1,%0"
		: "=r" (ret)
		: "rm" (val), "0" (-1));
	return ret + 1;
}

