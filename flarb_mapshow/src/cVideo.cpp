// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>

#include "flarb_mapshow/cVideo.h"
using namespace std;


// Functions executed at the beginning and end of the Application
bool cVideo::Create()
{
	// Init SDL
	if( SDL_Init( SDL_INIT_VIDEO ) != 0 )
		return false;

	// Init display
	if( (_display = SDL_SetVideoMode( 800, 600, 32, SDL_HWSURFACE | SDL_DOUBLEBUF )) == NULL)
		return false;

	// Init _sdl_colors
	for(int i = 0, offset = 0; i < VIDEO_COLOR_NUM; i++, offset += 3)
	{
		_sdl_colors[i] = SDL_MapRGB( 
							_display->format,
							VIDEO_COLOR_VALUE[offset + 0], 
							VIDEO_COLOR_VALUE[offset + 1],
							VIDEO_COLOR_VALUE[offset + 2]);
	}
	
	return true;
}


void cVideo::Destroy()
{
	// Quit SDL
	SDL_Quit();
}

void cVideo::Clear( uint32_t imageX, uint32_t imageY)
{
	// Fill the image with white, so we only have to write the black dots
	SDL_Rect rect = { 0, 0, (uint16_t)imageX, (uint16_t)imageY};
	SDL_FillRect( _display, &rect, _sdl_colors[COLOR_WHITE]);
}


void cVideo::Update()
{
	// Flip the buffer
	SDL_Flip( _display);
}


void cVideo::DrawPixel( int x, int y, enum VIDEO_COLOR color)
{
	__DrawPixel( x, y, _sdl_colors[color]);
}


void cVideo::__DrawPixel( int x, int y, uint32_t color)
{
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

		/*case 3: { // Slow 24-bpp mode, usually not used
			Uint8 *bufp;

			bufp = (Uint8 *)_display->pixels + y*_display->pitch + x;
			*(bufp+_display->format->Rshift/8) = R;
			*(bufp+_display->format->Gshift/8) = G;
			*(bufp+_display->format->Bshift/8) = B;
			break;
		}*/

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

