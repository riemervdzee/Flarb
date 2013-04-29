// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "ros/ros.h"

#include "flarb_img_mapbuilder/cMapbuilder.h"
using namespace std;


void cMapbuilder::ProcessMessage( const sensor_msgs::LaserScan &msg)
{
	_frame.GenerateFrame( msg);
}

void cMapbuilder::RegenerateImage()
{
	_image.ClearImage();
	_image.AddFramePoints( _frame);
}
