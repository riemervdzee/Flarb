#ifndef CLASS_ROSCOM_H
#define CLASS_ROSCOM_H

#include <vector>

#include "ros/ros.h"

// msgs
#include "flarb_msgs/DualMotorEncoder.h"
#include "flarb_msgs/DualMotorSpeed.h"
#include "flarb_msgs/Signal.h"

// Canbus protocol
#include "flarb_canbus/protocol/protocol.h"
#include "flarb_canbus/protocol/protocol_dual_dc_motor_driver.h"
#include "flarb_canbus/protocol/protocol_gpo_controller.h"

// Prototype class
struct cCanbus;


/*
 * Ros communication class
 */
class cRosCom
{
public:
	// C-tor
	cRosCom() : _devSpeedID( 0x101), _devGPIOID( 0x102), SwitchLeftRight( false), SwitchForwardBackward( false) {}

	// Functions executed at the beginning and end of the Application
	int Create( ros::NodeHandle *rosNode, cCanbus *canbus);
	int Destroy();

	// Publish a message from the canbus to ROS-topics
	void MessageReceived( const struct CanMessage &msg);

private:
	// Puts a speed message on the Canbus
	void SendSpeed( const flarb_msgs::DualMotorSpeedPtr msg);

	// Puts a GPO message on the Canbus
	void SendSignal( const flarb_msgs::Signal msg);
	
	// Process functions
	void ProcessHelloMessage( const struct CanMessage &canmessage);
	void ProcessDeviceSpeedMessage( const struct CanMessage &canmessage);


	// ID of every device we are connected to
	int _devSpeedID;
	int _devGPIOID;

	// Handle to the rosnode __ROSCOM IS NOT THE OWNER OF THIS OBJ__
	ros::NodeHandle* _rosNode;

	// Pointer towards the canbus obj __ROSCOM IS NOT THE OWNER OF THIS OBJ__
	cCanbus* _canbus;

	// Subscriber to /canbus/speed
	ros::Subscriber _subSpeed;

	// Subscriber to /canbus/signal
	ros::Subscriber _subSignal;

	// Publisher to /canbus/encoder
	ros::Publisher _pubEncoder;

	// To make things Leon-proof
	bool SwitchLeftRight;
	bool SwitchForwardBackward;
};

#endif // CLASS_ROSCOM_H

