// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>
#include <cmath>

#include "ros/ros.h"
#include "flarb_msgs/WaypointVector.h"

#include "flarb_keyboard/Graphics.h"
#include "flarb_keyboard/cController.h"
using namespace std;

// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	// Create window
	window_create( "FLARB - Keyboard input");

	// We publish to /steering/waypoint
	_pubVector = _rosNode.advertise<flarb_msgs::WaypointVector>( "/steering/waypoint", 1);

	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{

}

// Updates the controller obj
void cController::Update()
{
	// Set basic change of vector based on input
	InputUpdate();
	if( INPUT_LEFT)  x -= 0.06f;
	if( INPUT_RIGHT) x += 0.06f;
	if( INPUT_UP)    y += 0.06f;
	if( INPUT_DOWN)  y -= 0.06f;

	// Try to get the vector back to ( 0, 0)
	//if( x > 0) x -= 0.02f; else if( x < 0) x += 0.02f;
	//if( y > 0) y -= 0.02f; else if( y < 0) y += 0.02f;

	// Put a limit of the vector at 0.5
	float size = sqrt(x*x + y*y);
	if(size > 0.2f)
	{
		float L = 0.2f / size;
		x *= L;
		y *= L;
	}

	// When near 0, clamp them to 0
	if( x > -0.01f && x < 0.01f) x = 0.f;
	if( y > -0.01f && y < 0.01f) y = 0.f;

	// Draw it
	drawSetColor( gRed);
	drawLine( 0, 0, x, y);

	// Send it
	flarb_msgs::WaypointVector msg;
	msg.x = x;
	msg.y = y;
	_pubVector.publish( msg);

	// Flip the window
	window_flip();
}
