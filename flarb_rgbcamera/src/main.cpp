// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_rgbcamera/cController.h"

using namespace std;

/*
 * Main entry of the TrafficController program
 */
int main(int argc, char **argv)
{
	// Pass main-arguments to ros, third argument is node-name (TODO add support for two cameras)
	ros::init( argc, argv, "rgbcamera");

	// cController shizzle
	cController controller;
	controller.Create();
	
	// Update timer
	ros::Rate loop_rate( 2);

	// Loop
	while (ros::ok())
	{
		controller.Update();

		// Only use if we have callbacks/services
		// ros::spinOnce();

		// Sleep for a little while
		loop_rate.sleep();
	}

	controller.Destroy();

	// 
	return 0;
}
