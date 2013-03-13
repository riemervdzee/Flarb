#ifndef CLASS_CAMERA_H
#define CLASS_CAMERA_H

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"

// mmap buffer struct
struct mmap_buffer {
	void   *start;
	size_t length;
};

/*
 * Main controller class of the rgb camera
 */
class cCamera
{
public:
    // Constructor
    cCamera() : _width( 0), _height( 0), _mmBuffers( NULL), _mmBuffersAmount( 0),
				_tmp_buffer(NULL), _count( 0) {}

    // Functions executed at the beginning and end of the Application
    bool Create( const char* DevicePath, int width, int height);
    void Destroy();

    // Get an image from the camera
    sensor_msgs::CompressedImage getImage();

protected:
	// Prototypes
	void deviceInit(void);
	void jpegWrite(unsigned char* img);
	void mainLoop(void);
	int frameRead(void);

	// Char array of the device-path (DOES NOT OWN THE DATA!)
	const char* _devicePath;

	// File handle to the camera device
	int _fileHandle;

	// Width and height of the images gained by the camera
	int _width;
	int _height;

	// Our first mmap'ed buffers, with these we can get data from the cam
	mmap_buffer* _mmBuffers;
	int _mmBuffersAmount;

	// Temporary buffer for rgb values
	unsigned char* _tmp_buffer;

	// The jpeg compressed buffer
	//std::vector<uint8_t> my_buffer;
	//jpeg_destination_mgr dest;

	// Sequence ID
	unsigned int _count;
};

#endif // CLASS_CAMERA_H
