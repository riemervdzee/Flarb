#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include "ros/ros.h"
#include "flarb_mapbuilder/MapImage.h"

// forward declaration of cController
class cController;

/*
 * 
 */
class cRosCom
{
public:
	// Functions executed at the beginning and end of the Application
	int Create( ros::NodeHandle *rosNode, cController *controller);
	int Destroy();

private:
	// We received a laserscan frame
	void ImgCallback( const flarb_mapbuilder::MapImage msg);


	// Pointer to the cController class _NOTE: WE ARE NOT THE OWNER_
	cController *_controller;

	// This node is subscribed to the topic "map"
	ros::Subscriber _subMap;
};

#endif // CLASS_FRAME_H

