#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include <string>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "flarb_msgs/DualMotorSpeed.h"
#include "flarb_msgs/VDState.h"
#include "flarb_simulation/cMap.h"

// forward declaration of cController
class cController;
class cCar;

/*
 * Handles all communication with ROS
 */
class cRosCom
{
public:
	// Functions executed at the beginning and end of the Application
	int Create( ros::NodeHandle *rosNode, cController *controller, cCar *car, int hz, std::string str);
	int Destroy();

	// Resends the ActionString
	void Reset();

	// Publishes a WaypointVector
	void PublishLaserScan( const cMap &map);

private:
	// We received a map
	//void MapbuildCallback( const flarb_mapbuilder::MapList msg);
	void SpeedCallback( const flarb_msgs::DualMotorSpeed msg);

	// State callback
	bool StateCallback( flarb_msgs::VDState::Request &req, flarb_msgs::VDState::Response &res);


	// The actionstring we send to the controller
	std::string _str;
	
	// msg cache
	sensor_msgs::LaserScan _msg;

	// Pointer to the cController class _NOTE: WE ARE NOT THE OWNER_
	cController *_controller;

	// Pointer to the car
	cCar *_car;

	// Subscribers and publishers
	ros::Publisher  _pubSick;   //  "/sick/scan_filtered/"
	ros::Publisher  _pubSmart;  //  "/smartphone/input"
	ros::Subscriber _subSpeed;  //  "/canbus/speed/"

	// Service handle
	ros::ServiceServer _StateService;
};

#endif // CLASS_FRAME_H

