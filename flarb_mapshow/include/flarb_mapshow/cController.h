#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"



//#include "flarb_mapbuilder/Object.h"
#include "flarb_mapbuilder/Map.h"
//#include "flarb_img_controller/WaypointVector.h"

/*
 * Main controller class of the example node
 */
class cController
{
public:
	// Functions executed at the begining and end of the Application
	bool Create();
	void Destroy();
	void Update();

private:
	// Callbacks
	void ImgCallback( const flarb_mapbuilder::Map msg);
	//void WaypointCallback( const flarb_controller::WaypointVector msg);


	// Copy of the last flarb_controller::WaypointVector
	//flarb_controller::WaypointVector _lastVector;

	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// We are subscribed to topic "/map"
	ros::Subscriber _subImg;

	// Subscriber to "steering/waypoint"
	ros::Subscriber _subWaypoint;
};

#endif // CLASS_CONTROLLER_H
