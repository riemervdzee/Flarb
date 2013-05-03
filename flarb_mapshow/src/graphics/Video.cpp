/*
  Video.cpp -- Graphics back-end for tutorial purposes

  Copyright (c) 2011-2013 Riemer van der Zee <riemervdzee@gmail.com>

  This software is provided 'as-is', without any express or implied
  warranty. In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

     1. The origin of this software must not be misrepresented; you must not
        claim that you wrote the original software. If you use this software
        in a product, an acknowledgment in the product documentation would be
        appreciated but is not required.

     2. Altered source versions must be plainly marked as such, and must not be
        misrepresented as being the original software.

     3. This notice may not be removed or altered from any source
        distribution.
 */

#include "flarb_mapshow/Graphics.h"
#include <cmath>
#include <GL/gl.h>
using namespace std;

/*
 * Sets the current drawing color
 */
void drawSetColor ( gColor col)
{
	// Variables
	float R, G, B;

	// Determine which color to use
	switch ( col)
	{
	case gWhite:
		R = G = B = 1.0f;
		break;
	case gBlack:
		R = G = B = 0.0f;
		break;
	case gRed:
		R = 1; G = B = 0.0f;
		break;
	case gBlue:
		R = G = 0.0f; B = 1.0f;
		break;
	default:
		return;
		break;
	}

	// Set the actual color
	glColor3f( R, G, B);
}


/*
 * Draws a pixel. note: that the size might actually be bigger than a pixel
 */
void drawPixel( float x, float y)
{
	glBegin( GL_POINTS);
		glVertex2f( x, y);
	glEnd();
}

/*
 * Draws a smooth line between the two given points
 */
void drawLine( float x0, float y0, float x1, float y1)
{
	glBegin(GL_LINES);
		glVertex2f( x0, y0);
		glVertex2f( x1, y1);
	glEnd();

}

/*
 * Draws a hollow segmented circle at the given position and radius
 */
void drawCircle( float cx, float cy, float r, int num_segments)
{
	float theta = (2 * M_PI) / float(num_segments);
	float c = cosf( theta); //precalculate the sine and cosine
	float s = sinf( theta);

	float x = r;//we start at angle = 0
	float y = 0;

	glPushMatrix();
	glTranslatef( cx, cy, 0);

	glBegin( GL_LINE_LOOP);
	for(int ii = 0; ii < num_segments; ii++)
	{
		glVertex2f( x, y);//output vertex

		//apply the rotation matrix
		float t = x;
		x = c * x - s * y;
		y = s * t + c * y;
	}
	glEnd();

	glPopMatrix();
}


