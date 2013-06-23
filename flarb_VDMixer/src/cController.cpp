// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>
#include <cmath>

#include "ros/ros.h"
#include "flarb_VDMixer/cController.h"

// Used in program
#define DEG2RAD M_PI/180.0

/*
 * Functions executed at the beginning of the Node
 */
bool cController::Create()
{
	// Set up service
	StateService = _rosNode.advertiseService<cController>( "/vdmixer/state", &cController::StateCallback, this);

	//Subscriber for data 
	Inclino = _rosNode.subscribe<flarb_msgs::Axis>("sensor/inclination", 1, &cController::axismsg, this);
	Compass = _rosNode.subscribe<flarb_msgs::Compass>("sensor/compass", 1, &cController::compassmsg, this);
	Encoder = _rosNode.subscribe<flarb_msgs::Encoder>("steering/encoder", 1, &cController::encodermsg, this);
	GGA     = _rosNode.subscribe<flarb_msgs::GGA>("NMEA/GGA",1, &cController::GGAmsg, this);
	RMC     = _rosNode.subscribe<flarb_msgs::RMC>("NMEA/RMC",1, &cController::RMCmsg, this);
	return 0;
}

// Executed when the Node is exiting
void cController::Destroy()
{

}

/*
 * Generates a VDMixer/State message
 */
void cController::Update()
{

}

/**
 * Service handler
 */
bool cController::StateCallback( flarb_msgs::VDState::Request &req, flarb_msgs::VDState::Response &res)
{
	// Fill values
	//res.positionX
	//res.positionY
	res.distance = distance;
	res.axisX = message_axis.x * DEG2RAD;
	res.axisY = message_axis.y * DEG2RAD;
	res.axisZ = (2*M_PI)-(message_compass.angle * DEG2RAD);

	return true;
}

/*
 *	Callbacks
 */
void cController::axismsg(const flarb_msgs::Axis msgr)
{
	message_axis = msgr;
}

void cController::compassmsg(const flarb_msgs::Compass msgr)
{
	message_compass = msgr;
}

void cController::encodermsg(const flarb_msgs::Encoder msgr)
{
	message_encoder = msgr;

	// Increase distance
	distance += ((msgr.speed_left + msgr.speed_right) * 0.5f);
}

void cController::GGAmsg(const flarb_msgs::GGA msgr)
{
	message_gga = msgr;
}

void cController::RMCmsg(const flarb_msgs::RMC msgr)
{
	message_rmc = msgr;
}

