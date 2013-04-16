#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"
#include "flarb_mapbuilder/MapImage.h"
#include "flarb_mapshow/cVideo.h"


/*
 * Main controller class of the example node
 */
class cController
{
public:
	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

private:
	// Callback
	void ImgCallback( const flarb_mapbuilder::MapImage msg);

	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// We are subscribed to topic "/map"
	ros::Subscriber _subImg;
	
	// Video object
	cVideo _video;
};

#endif // CLASS_CONTROLLER_H
