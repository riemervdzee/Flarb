// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"
#include "flarb_msgs/PlantQualityRequest.h"
#include "flarb_controller/cRosCom.h"
#include "flarb_controller/cController.h"
using namespace std;


int cRosCom::Create( ros::NodeHandle *rosNode, cController *controller)
{
	// Set controller
	_controller = controller;

	// Subscribe to map
	_subMap = rosNode->subscribe<flarb_msgs::MapList>( "/map", 1, &cRosCom::MapCallback, this);
	
	// Subscribe to our smartphone input
	_subSmartphone = rosNode->subscribe<std_msgs::String>( "/smartphone/input", 1, &cRosCom::SmartphoneCallback, this);

	// We publish to /steering/waypoint
	_pubVector = rosNode->advertise<flarb_msgs::WaypointVector>( "/steering/waypoint", 1);

	// Publishing to "/plantquality/request"
	_pubPQRequest = rosNode->advertise<flarb_msgs::PlantQualityRequest>( "/plantquality/request", 5);

	// Our vdmixer
	_vdmixer = rosNode->serviceClient<flarb_msgs::VDState>("/vdmixer/state");

	return 0;
}

int cRosCom::Destroy()
{
	return 0;
}


// Publishes a WaypointVector
void cRosCom::PublishWaypoint( const flarb_msgs::WaypointVector &msg)
{
	_pubVector.publish( msg);
}


// Publishes a PlantQualityRequest
void cRosCom::PublishPlantQualityRequest( bool left, bool right)
{
	// Construct message
	flarb_msgs::PlantQualityRequest msg;
	msg.CheckLeft  = left;
	msg.CheckRight = right;

	// Publish it
	_pubPQRequest.publish( msg);
}


// Calls the vd state service
void cRosCom::GetVDState( flarb_msgs::VDState &data)
{
	_vdmixer.call( data);
}


// We received a map
void cRosCom::MapCallback( const flarb_msgs::MapList msg)
{
	// Pass the message to the main-controller
	_map.setMapList( msg);
	_controller->MapCallback( _map);
}


// We received orders from the smartphone
void cRosCom::SmartphoneCallback( const std_msgs::String msg)
{
	_controller->SmartphoneCallback( msg.data);
}
