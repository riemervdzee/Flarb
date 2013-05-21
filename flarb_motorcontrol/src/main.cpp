// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_motorcontrol/cController.h"
using namespace std;

// Node defines
#define NODE_NAME       "motorcontrol"  // Name of the module
#define NODE_FREQUENCY  30              // Amount of loops per second
#define NODE_CALLBACKS  1               // 0 = false, 1 = true. use when the node has services to offer

/*
 * Main entry of the mapshow node
 */
int main(int argc, char **argv)
{
	// Pass main-arguments to ros, third argument is node-name
	ros::init( argc, argv, NODE_NAME);

	// Create a cController object
	cController controller;
	controller.Create( NODE_FREQUENCY);

	// Update timer
	ros::Rate loop_rate( NODE_FREQUENCY);

	// Loop
	while (ros::ok())
	{
		// Only use if we have callbacks/services
#if NODE_CALLBACKS
		ros::spinOnce();
#endif

		// Let the controller update
		controller.Update();

		// Sleep for a little while
		loop_rate.sleep();
	}

	// Destroy the controller obj
	controller.Destroy();

	// return succes
	return 0;
}
