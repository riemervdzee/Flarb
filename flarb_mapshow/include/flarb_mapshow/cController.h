#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include <SDL/SDL.h>

#include "ros/ros.h"
#include "flarb_mapbuilder/MapImage.h"


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
	//
	void ImgCallback( const flarb_mapbuilder::MapImage msg);
	void DrawPixel2( int x, int y, bool value);
	void DrawPixel( int x, int y, Uint8 R, Uint8 G, Uint8 B);

	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// We are publishing shizzle
	ros::Subscriber _subImg;

	// Our SDL surface
	SDL_Surface *_display;
};

#endif // CLASS_CONTROLLER_H
