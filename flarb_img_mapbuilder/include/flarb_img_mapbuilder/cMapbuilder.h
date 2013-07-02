#ifndef CLASS_MAPBUILDER_H
#define CLASS_MAPBUILDER_H

#include "ros/ros.h"

#include "flarb_img_mapbuilder/cImage.h"
#include "flarb_img_mapbuilder/cFrame.h"


/*
 * 
 */
class cMapbuilder
{
public:
	void ProcessMessage( const sensor_msgs::LaserScan &msg);
	void RegenerateImage();

	cImage _image;

private:

	cFrame _frame;
};

#endif // CLASS_MAPBUILDER_H

