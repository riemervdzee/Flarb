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
	if( (_display = SDL_SetVideoMode( 1024, 768, 32, SDL_HWSURFACE | SDL_DOUBLEBUF )) == NULL)
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


/*
 * Clears a certain area
 */
void cVideo::Clear( uint32_t imageX, uint32_t imageY)
{
	// Fill the image with white, so we only have to write the black dots
	SDL_Rect rect = { 0, 0, (uint16_t)imageX, (uint16_t)imageY};
	SDL_FillRect( _display, &rect, _sdl_colors[COLOR_WHITE]);
}


/*
 * Flip the SDL buffer
 */
void cVideo::Update()
{
	// Flip the buffer
	SDL_Flip( _display);
}


/*
 * Draws a pixel at x/y with the color
 */
void cVideo::DrawPixel( int x, int y, enum VIDEO_COLOR color)
{
	__DrawPixel( x, y, _sdl_colors[color]);
}


/*
 * Actually draws the pixel
 */
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


/*
 * Draws a line at (x,y)0 to (x,y)1 with color
 */
void cVideo::DrawLine( int x0, int y0, int x1, int y1, enum VIDEO_COLOR color)
{
	// Check if in range
	// TODO maybe we need this as well?
	//if( !PixelInRange( x0, y0 ) && !PixelInRange( x1, y1 ))
	//	return;

	bool steep = abs(y1 - y0) > abs(x1 - x0);

	if ( steep) {
		std::swap( x0, y0);
		std::swap( x1, y1);
	}

	if ( x0 > x1) {
		std::swap( x0, x1);
		std::swap( y0, y1);
	}

	int deltax = x1 - x0;
	int deltay = abs(y1 - y0);
	int error = deltax / 2;
	int ystep;
	int y = y0;

	if (y0 < y1)
		ystep = +1;
	else
		ystep = -1;

	for ( int x = x0; x < x1; x++)
	{
		if ( steep)
			DrawPixel( y, x, color);
		else
			DrawPixel( x, y, color);

		error = error - deltay;

		if (error < 0) {
			y = y + ystep;
			error = error + deltax;
		}
	}
}

