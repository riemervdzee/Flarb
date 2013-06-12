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
	Inclino = _rosNode.subscribe<flarb_inclination::Axis>("sensor/inclination", 1, &cController::axismsg, this);
	Compass = _rosNode.subscribe<flarb_compass::Compass>("sensor/compass", 1, &cController::compassmsg, this);
	Encoder = _rosNode.subscribe<flarb_motorcontrol::Encoder>("steering/encoder", 1, &cController::encodermsg, this);
	GGA     = _rosNode.subscribe<flarb_gps::GGA>("NMEA/GGA",1, &cController::GGAmsg, this);
	RMC     = _rosNode.subscribe<flarb_gps::RMC>("NMEA/RMC",1, &cController::RMCmsg, this);
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
bool cController::StateCallback( flarb_VDMixer::State::Request &req, flarb_VDMixer::State::Response &res)
{
	// Fill values
	//res.positionX
	//res.positionY
	//res.distance
	res.axisX = message_axis.x * DEG2RAD;
	res.axisY = message_axis.y * DEG2RAD;
	res.axisZ = message_compass.north_angle * DEG2RAD;

	return true;
}

/*
 *	Callbacks
 */
void cController::axismsg(const flarb_inclination::Axis msgr)
{
	message_axis = msgr;
}

void cController::compassmsg(const flarb_compass::Compass msgr)
{
	message_compass = msgr;
}

void cController::encodermsg(const flarb_motorcontrol::Encoder msgr)
{
	message_encoder = msgr;
}

void cController::GGAmsg(const flarb_gps::GGA msgr)
{
	message_gga = msgr;
}

void cController::RMCmsg(const flarb_gps::RMC msgr)
{
	message_rmc = msgr;
}

