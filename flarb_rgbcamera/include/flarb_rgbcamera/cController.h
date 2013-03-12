#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"

#include "flarb_rgbcamera/cCamera.h"

/*
 * Main controller class of the rgb camera
 */
class cController
{
public:
    // Constructor
    cController(){}

    // Functions executed at the beginning and end of the Application
    bool Create();
    void Destroy();

    // Updates the Server
    void Update();

private:
	
	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// We are publishing shizzle
	ros::Publisher  _rosTopic;

	// Object which manages the cam for us
	cCamera _camera;
};

#endif // CLASS_CONTROLLER_H
