#ifndef CLASS_CAR_H
#define CLASS_CAR_H

#include "flarb_msgs/DualMotorSpeed.h"

// Gain of both engines per step
#define MOTOR_GAIN 0.4f

class cCar
{
public:
	// C-tor
	cCar( float _x, float _y, float _dir, float _factor) : x( _x), y( _y), 
		direction( _dir), factor( _factor), start_x( _x), start_y( _y), 
		start_direction( _dir), distance( 0), motor_l( 0), motor_r( 0) {}

	// Empty C-tor, which leaves the car unitialized..
	cCar(){}

	// Draws the car via the Graphics extension
	void Draw();

	// Update position
	void Update();

	// Resets the position and dir
	void Reset();

	// Sets the motor goal
	inline void setGoal( flarb_msgs::DualMotorSpeed _goal) {goal = _goal;}

	// Getters
	inline float getX()         { return x;}
	inline float getY()         { return y;}
	inline float getDirection() { return direction;}
	inline float getDistance()  { return distance;}
	inline float getFactor()    { return factor;}

private:
	// Vars
	float x, y;
	float direction; // In radians
	float factor;

	// Saved values
	float start_x, start_y, start_direction;

	// Distance travelled
	float distance;

	// Current strengths of the motor
	float motor_l;
	float motor_r;

	// Goal of the engines
	flarb_msgs::DualMotorSpeed goal;
};

#endif // CLASS_CAR_H
