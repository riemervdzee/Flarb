// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <vector>

#include "ros/ros.h"

#include "flarb_mapshow/cController.h"
#include "flarb_mapshow/Graphics.h"
using namespace std;


// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	// Subscribe to "/map"
	_subImg = _rosNode.subscribe<flarb_mapbuilder::Map>( "/map", 1, &cController::ImgCallback, this);

	// Subscribe "steering/waypoint"
	//_subWaypoint = _rosNode.subscribe<flarb_img_controller::WaypointVector>
	//				( "steering/waypoint", 1, &cController::WaypointCallback, this);

	// Init video
	window_create( "FLARB - Showing output topic \"/Map\"");

	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	// Deinit video object
	//_video.Destroy();
}

// Gets called X times per sec, to update the node
void cController::Update()
{
	InputUpdate();
	// TODO Use INPUT_STOP 
}

void cController::ImgCallback( const flarb_mapbuilder::Map msg)
{
	drawSetColor( gBlack);

	for( unsigned int i = 0; i < msg.list.size(); i++)
	{
		flarb_mapbuilder::Object obj = msg.list[i];
		drawCircle( obj.x, obj.y, obj.radius, 8);
		//drawPixel( obj.x, obj.y);
	}

	// Flip it
	window_flip();
}


/*void cController::WaypointCallback( const flarb_controller::WaypointVector msg)
{
	// Copy the waypoint
	_lastVector = msg;
}*/

