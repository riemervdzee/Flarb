#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"

#include "flarb_mapshow/cVideo.h"

#include "flarb_mapbuilder/MapImage.h"
#include "flarb_controller/WaypointVector.h"

/*
 * Main controller class of the example node
 */
class cController
{
public:
	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

private:
	// Callbacks
	void ImgCallback( const flarb_mapbuilder::MapImage msg);
	void WaypointCallback( const flarb_controller::WaypointVector msg);


	// Copy of the last flarb_controller::WaypointVector
	flarb_controller::WaypointVector _lastVector;

	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// We are subscribed to topic "/map"
	ros::Subscriber _subImg;

	// Subscriber to "steering/waypoint"
	ros::Subscriber _subWaypoint;

	// Video object
	cVideo _video;
};

#endif // CLASS_CONTROLLER_H
