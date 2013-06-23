// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <cmath>
#include <GL/gl.h>
#include "ros/ros.h"

#include "flarb_simulation/config.h"
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

void cCar::Update()
{
	// Step towards the wanted motor velocities
	// A tad crude, but works pretty well enough
	if( motor_l < goal.speed_left)
		motor_l += MOTOR_GAIN;
	else if( motor_l > goal.speed_left)
		motor_l -= MOTOR_GAIN;

	if( motor_r < goal.speed_right)
		motor_r += MOTOR_GAIN;
	else if( motor_r > goal.speed_right)
		motor_r -= MOTOR_GAIN;

	// Convert pulse motors to m velocities
	float Vl = motor_l * 0.001f;
	float Vr = motor_r * 0.001f;

	// Calc distance travelled
	distance += ((Vl + Vr) * 0.5) * factor;

	// Calculate the difference in direction and the speed (scalar)
	float w = (CAR_WHEEL_R / CAR_WHEEL_L) * ( Vl - Vr);
	float s = (CAR_WHEEL_R / 2) * (Vl + Vr);

	// Integrate velocity into the car state
	x += s * cos( direction);
	y += s * sin( direction);
	direction -= w; // Incorrect, but ignore

	// 0 <= _direction <= 2xPI
	if( direction > (2*M_PI))
		direction -= (2*M_PI);
	else if( direction < 0)
		direction += (2*M_PI);
}

// Resets the position and dir
void cCar::Reset()
{
	// Reset values
	x         = start_x;
	y         = start_y;
	direction = start_direction;
	distance  = 0.0f;
	motor_l   = 0.0f;
	motor_r   = 0.0f;

	// Reset message
	goal.speed_left  = 0.0f;
	goal.speed_right = 0.0f;
	goal.flags       = 0;
}
