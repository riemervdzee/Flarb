/*
 * 	Author: Daniel de Valk
 * 	Class: Movement in the controller
 *
 *	Expectation: ultra Slow!
 *	Calculate a straight line 
 */

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include "flarb_mapbuilder/MapImage.h"

int cTrendLine::Create(int x, int y, flarb_mapbuilder::MapImage msg)
{
	yAxisValuesSum = 0;
	xAxisValuesSum = 0;
	//arrray Max points from laser
	int yAxisValues[1081];
    int xAxisValues[1081];
	int ptr = 0;

	//from here till line 52 must be rebuild to levels
	//now he tries the whole level(Image)	
	
// Iterator
	vector<uint8_t>::const_iterator itr = msg.data.begin();
	
	// Write the image to the buffer
	for( int y = 0; y < 512; y++)
	{
		for( int x = 0; x < (512 / 8); x++)
		{
			//Get value and advance the iterator
			int val = *(itr);
			itr++;
			
			//note all the x and y locations and count them
			for(int i = 0; i < 7; i++){
				if(1 == (val & ( 1 << i )))
				{
					yAxisValues[ptr] = y;
					yAxisValuesSum += y;
					xAxisValues[ptr] = x * 8;
					xAxisValuesSum += x;
					ptr++;
				}		
			}			
		}
	}
	
	//unnecessary pointer
	count = ptr;
	//sum of x and y
	xxSum = 0;
    xySum = 0;
	
	
	for (int i = 0; i < count; i++)
    {
        xySum += ( xAxisValues[i] * yAxisValues[i] );
        xxSum += ( xAxisValues[i] * xAxisValues[i] );
    }

	Slope = calculateSlope();
    Intercept = calculateIntercept();
    Start = calculateStart();
    End = calculateEnd();
    return 1;
}
/*
 *	Calculate slope
 */
int cTrendLine::calculateSlope()
{
    try
    {
        return ((count * xySum) - (xAxisValuesSum * yAxisValuesSum)) / ((count * xxSum) - ( xAxisValuesSum * xAxisValuesSum));
    }
    catch (int e)
    {
        return 0;
    }
}


/*
 *	Calculate the interception(Offset)
 */
int cTrendLine::calculateIntercept()
{
    return ( yAxisValuesSum - (Slope * xAxisValuesSum)) / count;
}

/*
 *	Calculate start
 */
int cTrendLine::calculateStart()
{
    return ( Slope * xAxisValues[0]) + Intercept;
}

/*
 *	Calculate the end of our trendline
 */
int cTrendLine::calculateEnd()
{
    return ( Slope * xAxisValues[ptr]) + Intercept;
}

/*
 *	give x, y, msg the Pixel and returns
 *	1 == available
 *	0 == none
 */
int cTrendLine::getPixel(int x , int y, flarb_mapbuilder::MapImage msg)
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

