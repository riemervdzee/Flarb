#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include <vector>

#include "ros/ros.h"

#include "flarb_canbus/cCanbus.h"
#include "flarb_canbus/CanMessage.h"
#include "flarb_canbus/CanSubscribe.h"

/*
 * Helper struct for topics we should publish
 * note: first ros:Publisher, then int id (from big to small).
 */
struct sRosComPublishEntry{
	ros::Publisher	topic;
	int				id;
};

/*
 * Ros communication class
 */
class cRosCom
{
public:
	// Functions executed at the beginning and end of the Application
	int Create( ros::NodeHandle *rosNode, cCanbus *canbus);
	int Destroy();

private:
	// Puts a message on the Canbus
	void SendCallback( const flarb_canbus::CanMessageConstPtr &msg);

	// We have a new Subscriber!
	bool SubscribeCallback( flarb_canbus::CanSubscribe::Request  &req,
							flarb_canbus::CanSubscribe::Response &res);

	// Publish a message from the canbus to ROS-topics
	void PublishMessage( const struct CanMessage &msg);


	// Handle to the rosnode __ROSCOM IS NOT THE OWNER OF THIS OBJ__
	ros::NodeHandle* _rosNode;

	// Pointer towards the canbus obj __ROSCOM IS NOT THE OWNER OF THIS OBJ__
	cCanbus* _canbus;

	// Subscriber to /canbus/send sends messages posted here
	ros::Subscriber _canSend;

	// Provides service so other Ros-Nodes can subscribe to certain Canbus id messages
	ros::ServiceServer _srvSubscribe;
	
	// Big sorted array of: id => ros::Subscriber ;
	std::vector< sRosComPublishEntry> _PublishEntries;
};

#endif // CLASS_ROSCOM_H

