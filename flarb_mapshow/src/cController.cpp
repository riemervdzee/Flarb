// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <vector>

#include "ros/ros.h"

#include "flarb_mapshow/cController.h"
using namespace std;


// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	// Subscribe to "/map"
	_subImg = _rosNode.subscribe<flarb_mapbuilder::MapImage>( "/map", 1, &cController::ImgCallback, this);

	// Subscribe "steering/waypoint"
	_subWaypoint = _rosNode.subscribe<flarb_controller::WaypointVector>
					( "steering/waypoint", 1, &cController::WaypointCallback, this);

	// Init video object
	_video.Create();

	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	// Deinit video object
	_video.Destroy();
}

void cController::ImgCallback( const flarb_mapbuilder::MapImage msg)
{
	// Clear screen
	_video.Clear( msg.imageX, msg.imageY);

	// Write the image to the buffer the hard way
	for( int y = 0, offset = 0; y < msg.imageY; y++)
	{
		for( int x = 0; x < (msg.imageX / 8); x++, offset++)
		{
			// Get value and advance the iterator
			int val = msg.data.at( offset);

			// Only execute if the byte ain't zero
			if( val != 0)
			{
				// Draw the pixels
				if( val & ( 1 << 0 )) _video.DrawPixel( x * 8 + 0, y, COLOR_BLACK);
				if( val & ( 1 << 1 )) _video.DrawPixel( x * 8 + 1, y, COLOR_BLACK);
				if( val & ( 1 << 2 )) _video.DrawPixel( x * 8 + 2, y, COLOR_BLACK);
				if( val & ( 1 << 3 )) _video.DrawPixel( x * 8 + 3, y, COLOR_BLACK);
				if( val & ( 1 << 4 )) _video.DrawPixel( x * 8 + 4, y, COLOR_BLACK);
				if( val & ( 1 << 5 )) _video.DrawPixel( x * 8 + 5, y, COLOR_BLACK);
				if( val & ( 1 << 6 )) _video.DrawPixel( x * 8 + 6, y, COLOR_BLACK);
				if( val & ( 1 << 7 )) _video.DrawPixel( x * 8 + 7, y, COLOR_BLACK);
			}
		}
	}

	// Draw last vector
	_video.DrawLine( msg.cameraX, msg.cameraY, msg.cameraX + _lastVector.x, msg.cameraY + _lastVector.y, COLOR_RED);
	_video.DrawPixel( msg.cameraX, msg.cameraY, COLOR_BLUE);

	// Flip it
	_video.Update();
}


void cController::WaypointCallback( const flarb_controller::WaypointVector msg)
{
	// Copy the waypoint
	_lastVector = msg;
}

