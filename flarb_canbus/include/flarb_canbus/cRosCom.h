#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include "ros/ros.h"

#include "flarb_canbus/cCanbus.h"
#include "flarb_canbus/CanMessage.h"
#include "flarb_canbus/CanSubscribe.h"

/*
 * Ros communication class
 */
class cRosCom
{
public:
	// Functions executed at the beginning and end of the Application
	int Create( ros::NodeHandle &_rosNode, const cCanbus &canbus);
	int Destroy();

private:
	// Function which gets called to send a package
	void SendCallback( const flarb_canbus::CanMessageConstPtr &msg);

	//
	//void SubscribeCallback( const flarb_canbus::CanMessageConstPtr &msg);

	// Pointer towards the canbus obj __ROSCOM IS NOT THE OWNER OF THIS OBJ__
	cCanbus* _canbus;

	// Subscriber to /canbus/send sends messages posted here
	ros::Subscriber _canSend;

	// Provides service so other Ros-Nodes can subscribe to certain Canbus id messages
	ros::ServiceServer _srvSubscribe;
	
	// TODO big sorted array of: id => ros::Subscriber ;
};

#endif // CLASS_ROSCOM_H

