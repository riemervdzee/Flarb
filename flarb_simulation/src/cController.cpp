// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_simulation/cController.h"
#include "flarb_simulation/Graphics.h"
using namespace std;

// Functions executed at the beginning and end of the Node
bool cController::Create( int hz)
{
	// Init video
	window_create( "FLARB - Showing output topic \"/Map\"");

	// Init map
	_map.Create();

	//
	_roscom.Create( &_rosNode, this, hz);

	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	_map.Destroy();
}

// Updates the controller obj
void cController::Update()
{
	// Draw map
	drawSetColor( gBlack);
	_map.Draw();

	// Draw player
	_car.Draw();

	// Flip it
	window_flip();

	// Scan!
	_roscom.PublishLaserScan( _car, _map);
}
