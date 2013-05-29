// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include <GL/gl.h>
#include "ros/ros.h"

#include "flarb_simulation/Graphics.h"
#include "flarb_simulation/cCar.h"
using namespace std;

void cCar::Draw()
{
	drawSetColor( gBlue);
	drawQuad( x, y, CAR_HEIGHT, CAR_WIDTH, direction);

	drawSetColor( gRed);
	glPushMatrix();
	glTranslatef( x, y, 0);
	glRotatef( direction * (180/ M_PI), 0.0f, 0.0f, 1.0f);
	drawLine( 0.0f, 0.0f, CAR_HEIGHT / 2, 0.0f);
	glPopMatrix();
}

