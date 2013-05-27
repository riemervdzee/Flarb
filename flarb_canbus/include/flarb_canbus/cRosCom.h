#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include <vector>

#include "ros/ros.h"

// msgs
#include "flarb_canbus/DualMotorEncoder.h"
#include "flarb_canbus/DualMotorSpeed.h"

// Canbus protocol
#include "flarb_canbus/protocol/protocol.h"
#include "flarb_canbus/protocol/protocol_dual_dc_motor_driver.h"

// Prototype class
struct cCanbus;


/*
 * Ros communication class
 */
class cRosCom
{
public:
	// C-tor
	cRosCom() : _devSpeedID( -1) {}

	// Functions executed at the beginning and end of the Application
	int Create( ros::NodeHandle *rosNode, cCanbus *canbus);
	int Destroy();

	// Publish a message from the canbus to ROS-topics
	void MessageReceived( const struct CanMessage &msg);

private:
	// Puts a speed message on the Canbus
	void SendSpeed( const flarb_canbus::DualMotorSpeedPtr msg);
	
	// Process functions
	void ProcessHelloMessage( const struct CanMessage &canmessage);
	void ProcessDeviceSpeedMessage( const struct CanMessage &canmessage);

	// ID of every device we are connected to
	int _devSpeedID;

	// Handle to the rosnode __ROSCOM IS NOT THE OWNER OF THIS OBJ__
	ros::NodeHandle* _rosNode;

	// Pointer towards the canbus obj __ROSCOM IS NOT THE OWNER OF THIS OBJ__
	cCanbus* _canbus;

	// Subscriber to /canbus/speed sends messages posted here
	ros::Subscriber _subSpeed;

	// Publisher to /canbus/encoder
	ros::Publisher _pubEncoder;
};

#endif // CLASS_ROSCOM_H

