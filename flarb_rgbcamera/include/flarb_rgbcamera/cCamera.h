#ifndef CLASS_CAMERA_H
#define CLASS_CAMERA_H

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"

/*
 * Main controller class of the rgb camera
 */
class cCamera
{
public:
    // Constructor
    cCamera() {}

    // Functions executed at the beginning and end of the Application
    bool Create( const char* device);
    void Destroy();

    // Get an image from the camera
    sensor_msgs::CompressedImage getImage();

private:

	// Sequence ID
	unsigned int _count;

	
};

#endif // CLASS_CAMERA_H
