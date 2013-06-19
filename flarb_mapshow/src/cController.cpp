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
	// Subscribe to topics
	_subMap      = _rosNode.subscribe<flarb_msgs::MapList>
					( "/map", 1, &cController::MapbuildCallback, this);
	_subRaw      = _rosNode.subscribe<sensor_msgs::LaserScan>
					( "/sick/scan", 1, &cController::RawCallback, this);
	_subFiltered = _rosNode.subscribe<sensor_msgs::LaserScan>
					( "/sick/scan_filtered", 1, &cController::FilterCallback, this);
	_subWaypoint = _rosNode.subscribe<flarb_msgs::WaypointVector>
					( "steering/waypoint", 1, &cController::WaypointCallback, this);

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

	// Quit on X click
	if( INPUT_STOP)
		ros::shutdown();

	// Togglers
	if(INPUT_Z) {
		INPUT_Z = false;
		DrawMap = !DrawMap;
	}
	if(INPUT_X) {
		INPUT_X = false;
		DrawRaw = !DrawRaw;
	}
	if(INPUT_C) {
		INPUT_C = false;
		DrawFilter = !DrawFilter;
	}
}

void cController::Draw()
{
	// Draw the Map
	if( DrawMap)
	{
		drawSetColor( gBlack);
		for( unsigned int i = 0; i < _lastMap.list.size(); i++)
		{
			flarb_msgs::Object obj = _lastMap.list[i];
			drawCircle( obj.x, obj.y, obj.radius, 8);
		}
	}

	// Draw Raw
	if( DrawRaw)
	{
		drawSetColor( gRed);
		DrawLaserScan( _lastLaserRaw);
	}

	// Draw Filtered
	if( DrawFilter)
	{
		drawSetColor( gBlue);
		DrawLaserScan( _lastLaserFiltered);
	}

	// Draw the velocity vector
	if( DrawMap)
	{
		drawSetColor( gRed);
		drawLine( 0, 0, _lastVector.x, _lastVector.y);
	}

	// Draw camera position
	drawSetColor( gBlue);
	drawPixel( 0, 0);

	// Flip it
	window_flip();
}

void cController::DrawLaserScan( const sensor_msgs::LaserScan &msg)
{
	// Vars
	float x_old = -1;
	float y_old = -1;
	bool  first = true;
	float angle = msg.angle_min;

	// Go through all points
	for( unsigned int i = 0; i < msg.ranges.size(); i++, angle += msg.angle_increment)
	{
		const float range = msg.ranges.at( i);

		// Is the value out of range? continue
		if( range < msg.range_min || range > msg.range_max)
			continue;

		float x = cos( angle) * range;
		float y = sin( angle) * range;

		// If this ain't the first pixel, draw it. otherwise skip
		if( !first)
		{
			// If the distance between points ain't too much, draw a line otherwise a dot
			if( ((x-x_old)*(x-x_old) + (y-y_old)*(y-y_old)) < OBJECT_DISTANCE_MAX)
				drawLine( x_old, y_old, x, y);
			else
				drawPixel( x, y);
		}
		else
			first = false;

		// Set old points
		x_old = x;
		y_old = y;
	}
}

void cController::MapbuildCallback( const flarb_msgs::MapList msg)
{
	_lastMap = msg;
	Draw();
}

void cController::FilterCallback( const sensor_msgs::LaserScan msg)
{
	_lastLaserFiltered = msg;
	if( !DrawMap) Draw();
}

void cController::RawCallback( const sensor_msgs::LaserScan msg)
{
	_lastLaserRaw = msg;
	if( !DrawMap && !DrawFilter) Draw();
}


void cController::WaypointCallback( const flarb_msgs::WaypointVector msg)
{
	_lastVector = msg;
}

