#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "flarb_mapbuilder/cMapbuilder.h"

/*
 * 
 */
class cRosCom
{
public:
	// Functions executed at the beginning and end of the Application
	int Create( ros::NodeHandle *rosNode);
	int Destroy();

private:
	// We received a laserscan frame
	void ScanCallback( const sensor_msgs::LaserScan msg);


	// This node is subscribed to the topics "sick/scan" and "vdmixer/state"
	ros::Subscriber _subSicklaser;
	//ros::Subscriber _subVDMixer; TODO decide whether we should change this to a service?
	
	// This node publishes to the topic "map"
	ros::Publisher _pubMap;
	
	//
	cMapbuilder _map;
};

#endif // CLASS_FRAME_H

