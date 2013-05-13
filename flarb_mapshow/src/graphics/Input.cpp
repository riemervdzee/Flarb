/*
  Input.cpp -- Handles basic input handling based on SDL - Source

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


// Include SDL
#include <SDL/SDL.h>

// All handled inputs
bool INPUT_STOP = false; // Should we stop the simulation?
bool INPUT_Z    = false;
bool INPUT_X    = false;
bool INPUT_C    = false;

// Private event struct
SDL_Event event;

/*
 * Basic inputUpdate
 */
void InputUpdate()
{
	// Loop through all new events
	while ( SDL_PollEvent(&event) )
	{
		switch( event.type )
		{
		// Case keyboard down
		case SDL_KEYDOWN:
			switch( event.key.keysym.sym )
			{
				// Toggle keys
				case SDLK_z:   INPUT_Z  = true; break;
				case SDLK_x:   INPUT_X  = true; break;
				case SDLK_c:   INPUT_C  = true; break;

				// Escape button
				case SDLK_ESCAPE:	INPUT_STOP	= true; break;
				default:			break;
			}
			break;

		// Case UP event
		/*case SDL_KEYUP:
			switch( event.key.keysym.sym )
			{
				// Toggle keys
				case SDLK_z:   INPUT_Z  = true; break;
				case SDLK_x:   INPUT_X  = true; break;
				case SDLK_c:   INPUT_C  = true; break;

				default:			break;
			}
			break;*/

		// Did we receive any stop-messages from the OS?
		case SDL_QUIT:
			INPUT_STOP = true;
			break;

		default:
			break;
		}
	}
}

