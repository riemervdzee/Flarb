#ifndef CLASS_CAR_H
#define CLASS_CAR_H

#include "flarb_simulation/config.h"

class cCar
{
public:
	// C-tor
	cCar() : x( CAR_POS_X), y( CAR_POS_Y), direction( CAR_DIR) {}

	// Functions executed at the begining and end of the Application
	//bool Create();
	//void Destroy();

	// Draws the map via Graphics extension
	void Draw();

	// Vars
	float x, y;
	float direction; // In radians
};

#endif // CLASS_CAR_H
