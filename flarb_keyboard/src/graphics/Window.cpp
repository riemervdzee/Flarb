/*
  Window.cpp -- Graphics back-end for tutorial purposes

  Copyright (c) 2011-2013 Riemer van der Zee <riemervdzee@gmail.com>

  This software is provided 'as-is', without any express or implied
  warranty. In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

     1. The origin of this software must not be misrepresented; you must not
        claim that you wrote the original software. If you use this software
        in a product, an acknowledgment in the product documentation would be
        appreciated but is not required.

     2. Altered source versions must be plainly marked as such, and must not be
        misrepresented as being the original software.

     3. This notice may not be removed or altered from any source
        distribution.
 */


#include <SDL/SDL.h>
#include <GL/gl.h>
using namespace std;

/*
 * Creates a window
 */
bool window_create( const char* Caption)
{
	// Initialize SDL with video support
	if( SDL_Init( SDL_INIT_VIDEO ) != 0 )
	{
		printf( "Could not initialize SDL. Quitting...\n");
		return true;
	}

	// Let SDL quit when we finish this program
	atexit( SDL_Quit);

	// All values are "at least"
	SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 );
	SDL_GL_SetAttribute( SDL_GL_RED_SIZE,	5);
	SDL_GL_SetAttribute( SDL_GL_GREEN_SIZE,	5);
	SDL_GL_SetAttribute( SDL_GL_BLUE_SIZE,	5);

	// Create video context with OpenGL
	if( SDL_SetVideoMode( 800, 600, 0, SDL_OPENGL ) != NULL)
	{
		// OpenGL extra things
		glShadeModel( GL_SMOOTH);
		glClearColor( 1, 1, 1, 1);

		glViewport( 0, 0, 800, 600);
		glMatrixMode( GL_PROJECTION);
		glLoadIdentity();
		glOrtho( -2, 2, -2, 2, -1, 1);
		glMatrixMode( GL_MODELVIEW);
		glLoadIdentity();

		glEnable( GL_POINT_SMOOTH);
		//glPointSize( 2.0);
	}
	else
	{
		// Failed initializing OpenGL
		printf( "Could not initialize OpenGL, make sure you use the latest video drivers");
		return true;
	}

	// Set the title
	SDL_WM_SetCaption( Caption, NULL);

	// Return
	return false;
}

/*
 * Flips buffers
 */
void window_flip()
{
	SDL_GL_SwapBuffers();
	glLoadIdentity();
	glClear( GL_COLOR_BUFFER_BIT );
}

