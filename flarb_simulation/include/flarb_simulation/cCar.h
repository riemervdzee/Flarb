#ifndef CLASS_CAR_H
#define CLASS_CAR_H

#include "flarb_simulation/config.h"
#include "flarb_canbus/DualMotorSpeed.h"

// Gain of both engines per step
#define MOTOR_GAIN 0.2f

class cCar
{
public:
	// C-tor
	cCar() : x( CAR_POS_X), y( CAR_POS_Y), direction( CAR_DIR), motor_l( 0), motor_r( 0) {}

	// Functions executed at the begining and end of the Application
	//bool Create();
	//void Destroy();

	// Draws the map via Graphics extension
	void Draw();

	//
	void Update();

	// Vars
	float x, y;
	float direction; // In radians

	// Current strengths of the motor
	float motor_l;
	float motor_r;

	//
	flarb_canbus::DualMotorSpeed goal;
};

#endif // CLASS_CAR_H
