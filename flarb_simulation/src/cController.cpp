// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <string>
#include <GL/gl.h>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "ros/ros.h"

#include "flarb_simulation/cController.h"
#include "flarb_simulation/Graphics.h"
using namespace std;

// Functions executed at the beginning and end of the Node
bool cController::Create( int hz)
{
	// Init video
	window_create( "Simulator");

	// Read XML
	boost::property_tree::ptree pt;
	read_xml( "maps/map1.xml", pt);

	//
	float carX = pt.get<float> ("map.car.x");
	float carY = pt.get<float> ("map.car.y");
	float carD = pt.get<float> ("map.car.direction");
	string str = pt.get<string>("map.string");

	// TODO do something with the str
	cout << str << endl;

	// Init map
	_map.Create();
	BOOST_FOREACH( boost::property_tree::ptree::value_type &v,
			pt.get_child("map.plants_group"))
		_map.Add( v.second.data());

	// Init car
	_car = cCar( carX, carY, carD);

	// Get the roscommunication going
	_roscom.Create( &_rosNode, this, &_car, hz);

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
	// Get Input and handle them
	InputUpdate();
	if( INPUT_STOP)      // Quit on X click
		ros::shutdown();
	if( INPUT_R)         // Reset sim
		_car.Reset();

	// Update childs
	_car.Update();

	// Draw the simulation
	Draw();
}

// Make sure everything gets drawn
void cController::Draw()
{
	// Center camera on the car
	glPushMatrix();
	glTranslatef( -_car.getX(), -_car.getY(), 0);

	// Draw map
	drawSetColor( gBlack);
	_map.Draw();

	// Draw player
	_car.Draw();

	// Flip it
	window_flip();

	// Scan!
	_roscom.PublishLaserScan( _map);

	// Reset cam
	glPopMatrix();
}
