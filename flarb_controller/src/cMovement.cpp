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
			countParticles += getPixel(x,y, &msg);
		}
	}
	return countParticles;
}

/*
 *	give x, y, msg the Pixel and returns
 *	1 == available
 *	0 == none
 */
int cMovement::getPixel(int x , int y, const flarb_mapbuilder::MapImage &msg)
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







