#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include "ros/ros.h"

#include "flarb_img_controller/WaypointVector.h"
#include "flarb_img_mapbuilder/MapImage.h"

// forward declaration of cController
class cController;


/*
 * Handles all communication with ROS
 */
class cRosCom
{
public:
	// Functions executed at the beginning and end of the Application
	int Create( ros::NodeHandle *rosNode, cController *controller);
	int Destroy();

	// Publishes a WaypointVector
	void PublishWaypoint( const flarb_img_controller::WaypointVector &msg);

private:
	// We received a laserscan frame
	void ImgCallback( const flarb_img_mapbuilder::MapImage msg);


	// Pointer to the cController class _NOTE: WE ARE NOT THE OWNER_
	cController *_controller;

	// This node is subscribed to the topic "map"
	ros::Subscriber _subMap;

	// This node is subscribed to the topic "map"
	ros::Publisher _pubVector;
};

#endif // CLASS_FRAME_H

