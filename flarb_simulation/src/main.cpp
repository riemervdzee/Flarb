// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_simulation/cController.h"
using namespace std;

// Node defines
#define NODE_NAME       "simulation" // Name of the module
#define NODE_FREQUENCY  30           // Amount of loops per second
#define NODE_CALLBACKS  1            // 0 = false, 1 = true. use when the node has services to offer

/*
 * Main entry of the example node
 */
int main(int argc, char **argv)
{
	// Pass main-arguments to ros, third argument is node-name
	ros::init( argc, argv, NODE_NAME);

	// We try to get the package path dynamically
	// We do this by getting the bin-path, and cut-off the "bin/simulation" part
	string arg(argv[0]);
	int size    = arg.size();
	int sizeEnd = string( "bin/simulation").size();
	string path = arg.substr( 0, size-sizeEnd);

	// Create a cController object
	cController controller;
	if( argc < 2)
		controller.Create( NODE_FREQUENCY, path, "map2.xml");
	else
		controller.Create( NODE_FREQUENCY, path, string(argv[1]));
	
	// Update timer
	ros::Rate loop_rate( NODE_FREQUENCY);

	// Loop
	while (ros::ok())
	{
		controller.Update();

		// Only use if we have callbacks/services
#if NODE_CALLBACKS
		ros::spinOnce();
#endif

		// Sleep for a little while
		loop_rate.sleep();
	}

	// Destroy the controller obj
	controller.Destroy();

	// return succes
	return 0;
}
