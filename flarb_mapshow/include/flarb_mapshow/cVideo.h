#ifndef CLASS_VIDEO_H
#define CLASS_VIDEO_H

#include <SDL/SDL.h>

#include "ros/ros.h"

/*
 * Possible colours
 */
enum VIDEO_COLOR {
	COLOR_WHITE,
	COLOR_BLACK,
	COLOR_RED,
	VIDEO_COLOR_NUM  // Amount of elements
};

// The actual color values
const uint8_t VIDEO_COLOR_VALUE[] = {
	/*  White  */   255, 255, 255,
	/*  Black  */     0,   0,   0,
	/*  Red    */   255,   0,   0,
};

/*
 * Main controller class of the example node
 */
class cVideo
{
public:
	// Functions executed at the beginning and end of the Application
	bool Create();
	void Destroy();

	void Clear( uint32_t imageX, uint32_t imageY);
	void Update();

	void DrawPixel( int x, int y, enum VIDEO_COLOR color);
	void __DrawPixel( int x, int y, uint32_t color);

private:
	// VIDEO_COLOR_VALUE translated to SDL values
	uint32_t _sdl_colors[VIDEO_COLOR_NUM];

	// Our SDL surface
	SDL_Surface *_display;
};

#endif // CLASS_VIDEO_H
