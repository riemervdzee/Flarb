#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"
#include "flarb_mobile/cServer.h"
#include "flarb_inclination/Axis.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>

/*
* Main controller class of the example node
*/
class cController
{
public:
    // Constructor
    cController() : _count( 0) {}

    // Functions executed at the beginning and end of the Application
    bool Create();
    void Destroy();
	//void Axismsg(flarb_inclination::Axis &msgr);
	
	void SendData();
    // Updates the Node
    void Update();

private:
	//Server object
	cServer _server;
	// Reference to the ros node handle
	ros::NodeHandle _rosNode;
	// We are publishing shizzle
	ros::Publisher _rosTopic;
	//Subscriber Message
	flarb_inclination::Axis message_axis;
	sensor_msgs::LaserScan lms;
	//Actual Subscriber
	ros::Subscriber _ax;
	ros::Subscriber _LMS;
	//Callback method
	void axismsg(const flarb_inclination::AxisConstPtr &msgr);
	void LMSmsg(const sensor_msgs::LaserScan::ConstPtr& scan);
	void packageReadout();	
	int _count;
};

#endif // CLASS_CONTROLLER_H
