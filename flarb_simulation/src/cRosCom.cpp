// Include order: cppstd, ROS, Boost, own-module includes
#include <cmath>
#include <iostream>

#include "ros/ros.h"

#include "flarb_simulation/cRosCom.h"
#include "flarb_simulation/cController.h"
using namespace std;


int cRosCom::Create( ros::NodeHandle *rosNode, cController *controller, cCar *car, int hz)
{
	// Set pointers
	_controller = controller;
	_car        = car;

	// Subscribers and publishers
	_pubSick  = rosNode->advertise<sensor_msgs::LaserScan>( "/sick/scan_filtered/", 1);
	_subSpeed = rosNode->subscribe<flarb_msgs::DualMotorSpeed>( "/canbus/speed/", 1, &cRosCom::SpeedCallback, this);

	// Set up service
	_StateService = rosNode->advertiseService<cRosCom>( "/vdmixer/state", &cRosCom::StateCallback, this);
	
	// Init LaserScan packet
	_msg.header.frame_id = "laser";
	_msg.range_min       = 0.01f;
	_msg.range_max       = 20.0f;
	_msg.scan_time       = 100.0/hz;
	_msg.angle_increment = 0.00436332f;
	_msg.angle_min       = -0.785398f;
	_msg.angle_max       =  3.926990f;
	_msg.time_increment  = _msg.scan_time/1081;
	_msg.ranges.resize(1081);

	return 0;
}

int cRosCom::Destroy()
{
	return 0;
}

/*
 * Standard info
 * min angle -0.785398
 * max angle  3.92699
 * angl. incr 0.00436332
 * time_increment 3.70028e-05
 * scan_time 0.04
 * n 1081
 *
 */
void cRosCom::PublishLaserScan( const cMap &map)
{
	// Update timestamp and sequence id
	ros::Time start = ros::Time::now();
	_msg.header.stamp = start;
	++_msg.header.seq;
	
	// Set starts
	// angle = car dir - 1/4PI (due min angle) - 1/2PI due the fact ROS wants
	// 0 radians to be poining upwards instead of to the right. bloody idiots
	float angle = _car->getDirection() - 2.35619449;
	tVector pos = tVector( _car->getX(), _car->getY());

	// Fill the ranges array
	for (int i = 0; i < 1081; i++)
	{
		_msg.ranges[i] = map.TestRayDistance( pos, angle);
		angle+= 0.00436332f;
	}

	_pubSick.publish(_msg);
}

void cRosCom::SpeedCallback( const flarb_msgs::DualMotorSpeed msg)
{
	_car->setGoal( msg);
}

bool cRosCom::StateCallback( flarb_msgs::State::Request &req, flarb_msgs::State::Response &res)
{
	// 0 <= _direction <= 2xPI
	float direction = _car->getDirection() - (M_PI/2);
	if( direction > (2*M_PI))
		direction -= (2*M_PI);
	else if( direction < 0)
		direction += (2*M_PI);

	res.positionX = _car->getX();
	res.positionY = _car->getY();
	res.distance  = _car->getDistance();
	res.axisX = 0;
	res.axisY = 0;
	res.axisZ = direction;

	return true;
}
