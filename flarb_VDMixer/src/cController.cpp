// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_VDMixer/cController.h"
#include "flarb_inclination/Axis.h"
#include "flarb_compass/Compass.h" 

// Functions executed at the beginning and end of the Node


/*
 *	Callback receiving message from MEAS Inclino
 */
void cController::axismsg(const flarb_inclination::Axis msgr)
{
	message_axis = msgr;
}

void cController::compassmsg(const flarb_compass::Compass msgr)
{
	message_north_angle = msgr;
}

void cController::phasemsg(const flarb_VDMixer::Phase msgr)
{
	message_phase = msgr;
}

bool cController::Create()
{
	//Subscriber for data 
	Compass = _rosNode.subscribe<flarb_compass::Compass>("sensor/compass", 1, &cController::compassmsg, this);
	//TODO: Modify Accelerator to standard
	//Accelerator = _rosNode.subscribe<flarb_inclination::Axis>("sensor/accelerator", 1, &cController::acceleratormsg, this);
	//TODO: Modify Gyro to standard
	//Gyro = _rosNode.subscribe<flarb_gyro::Axis>("sensor/gyro", 1, &cController::gyromsg, this);
	//TODO: Modify inclination to standard
	Inclination = _rosNode.subscribe<flarb_inclination::Axis>("sensor/inclination", 1, &cController::axismsg, this);
	//TODO: Modify PhaseControl to standard
	PhaseControl = _rosNode.subscribe<flarb_VDMixer::Phase>("sensor/phasecontrol", 1, &cController::phasemsg, this);
	ros::spin();
	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{

}

// Updates the controller obj
void cController::Update()
{

}
