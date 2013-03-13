// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"

#include "flarb_rgbcamera/cController.h"
#include "flarb_rgbcamera/cCamera.h"

// Functions executed at the beginning and end of the Application
bool cController::Create()
{
	// Topic name / buffer TODO get topic-name from argument?
	_rosTopic = _rosNode.advertise<sensor_msgs::CompressedImage>( "images", 10);

	// TODO get device path from arguments
	_camera.Create( "/dev/video1", 1280, 800);

	return true;
}


void cController::Destroy()
{
	_camera.Destroy();
}

// Get Image and publish it
void cController::Update()
{
	// Get image
	sensor_msgs::CompressedImage msg = _camera.getImage();

	// Publish
	_rosTopic.publish( msg);
}
