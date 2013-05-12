#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"


#include "sensor_msgs/LaserScan.h"
#include "flarb_mapbuilder/Map.h"
//#include "flarb_img_controller/WaypointVector.h"


// Maximum distance between points before considered separate objects
// This is in meters, squared. Keep this one the same as in /flarb_mapbuilder/ !
#define OBJECT_DISTANCE_MAX (0.10f * 0.10f)

/*
 * Main controller class of the example node
 */
class cController
{
public:
	// Constructor
	cController() : DrawMap( true), DrawRaw( false), DrawFilter( false) {}
	
	// Functions executed at the begining and end of the Application
	bool Create();
	void Destroy();
	void Update();

private:
	// Callbacks
	void MapbuildCallback ( const flarb_mapbuilder::Map msg);
	void RawCallback      ( const sensor_msgs::LaserScan msg);
	void FilterCallback   ( const sensor_msgs::LaserScan msg);
	void WaypointCallback ( const flarb_controller::WaypointVector msg);

	// Helper functions
	void Draw();
	void DrawLaserScan( const sensor_msgs::LaserScan &msg);


	// Togglers in drawing
	bool DrawMap;
	bool DrawRaw;
	bool DrawFilter;

	// Copy of the last few messages
	flarb_mapbuilder::Map  _lastMap;
	sensor_msgs::LaserScan _lastLaserRaw;
	sensor_msgs::LaserScan _lastLaserFiltered;
	flarb_controller::WaypointVector _lastVector;

	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// We are subscribed to quite a few topics
	ros::Subscriber _subMap;       // "map"
	ros::Subscriber _subWaypoint;  // "steering/waypoint"
	ros::Subscriber _subRaw;       // "sick/scan"
	ros::Subscriber _subFiltered;  // "sick/scan_filtered"
};

#endif // CLASS_CONTROLLER_H
