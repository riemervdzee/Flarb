// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_mapshow/cController.h"
using namespace std;

// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	// Subscribe to map
	_subImg = _rosNode.subscribe<flarb_mapbuilder::MapImage>( "map", 1, &cController::ImgCallback, this);
	//_pubMap = rosNode->advertise<>( "map", 1);

	// Init SDL
	if( SDL_Init( SDL_INIT_VIDEO ) != 0 )
		return false;

	if( (_display = SDL_SetVideoMode( 800, 600, 32, SDL_HWSURFACE | SDL_DOUBLEBUF )) == NULL)
		return false;

	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	// Quit SDL
	SDL_Quit();
}

void cController::ImgCallback( const flarb_mapbuilder::MapImage msg)
{
	// Fill the image with white, so we only have to write the black dots
	uint32_t color = SDL_MapRGB( _display->format, 255, 255, 255);
	SDL_Rect rect = { 0, 0, (uint16_t)msg.imageX, (uint16_t)msg.imageY};
	SDL_FillRect( _display, &rect, color);

	// Iterator
	vector<uint8_t>::const_iterator itr = msg.data.begin();

	// Write the image to the buffer the hard way
	for( int y = 0, offset = 0; y < 512; y++)
	{
		for( int x = 0; x < (512 / 8); x++, offset++)
		{
			// Get value and advance the iterator
			int val = msg.data.at( offset);

			// Only execute if the byte ain't zero
			if( val != 0)
			{
				// Draw the pixels
				DrawPixel2( x * 8 + 0, y, (bool)(val & ( 1 << 0 )));
				DrawPixel2( x * 8 + 1, y, (bool)(val & ( 1 << 1 )));
				DrawPixel2( x * 8 + 2, y, (bool)(val & ( 1 << 2 )));
				DrawPixel2( x * 8 + 3, y, (bool)(val & ( 1 << 3 )));
				DrawPixel2( x * 8 + 4, y, (bool)(val & ( 1 << 4 )));
				DrawPixel2( x * 8 + 5, y, (bool)(val & ( 1 << 5 )));
				DrawPixel2( x * 8 + 6, y, (bool)(val & ( 1 << 6 )));
				DrawPixel2( x * 8 + 7, y, (bool)(val & ( 1 << 7 )));
			}
		}
	}

	// Flip the buffer
	SDL_Flip( _display);
}

void cController::DrawPixel2( int x, int y, int value)
{
	uint8_t r, g, b;
	r = g = b = 0;
	
	if( value == 1)
		DrawPixel( x, y, r, g, b);
}

void cController::DrawPixel( int x, int y, Uint8 R, Uint8 G, Uint8 B)
{
	Uint32 color = SDL_MapRGB( _display->format, R, G, B);

	if ( SDL_MUSTLOCK( _display) ) {
		if ( SDL_LockSurface( _display) < 0 ) {
			return;
		}
	}

	switch ( _display->format->BytesPerPixel) 
	{
		case 1: {  /* Assuming 8-bpp */
			Uint8 *bufp;

			bufp = (Uint8 *)_display->pixels + y*_display->pitch + x;
			*bufp = color;
			break;
		}

		case 2: { /* Probably 15-bpp or 16-bpp */
			Uint16 *bufp;

			bufp = (Uint16 *)_display->pixels + y*_display->pitch/2 + x;
			*bufp = color;
			break;
		}

		case 3: { /* Slow 24-bpp mode, usually not used */
			Uint8 *bufp;

			bufp = (Uint8 *)_display->pixels + y*_display->pitch + x;
			*(bufp+_display->format->Rshift/8) = R;
			*(bufp+_display->format->Gshift/8) = G;
			*(bufp+_display->format->Bshift/8) = B;
			break;
		}

		case 4: { /* Probably 32-bpp */
			Uint32 *bufp;

			bufp = (Uint32 *)_display->pixels + y*_display->pitch/4 + x;
			*bufp = color;
			break;
		}
	}

	if ( SDL_MUSTLOCK( _display) ) {
		SDL_UnlockSurface( _display);
	}
}

// 
