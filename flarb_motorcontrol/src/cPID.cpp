#include <iostream>
#include "flarb_motorcontrol/cPID.h"
using namespace std;

cPID::cPID( int hz, enum Direction dir, float min, float max, float Kp, float Ki, float Kd) :
	Output( 0.f), ITerm( 0.f), lastInput( 0.f), SampleTime( 1.f / hz), 
	outMin( min), outMax( max), controllerDirection( dir)
{
	kp = Kp;
	ki = Ki * SampleTime;
	kd = Kd / SampleTime;

	cout << "kp " << kp << ", ki " << ki << ", kd " << kd << endl;
}

float cPID::Execute( float goal, float input)
{
	/*Compute all the working error variables*/
	float error = goal - input;
	ITerm+= (ki * error);
	if(ITerm > outMax) ITerm= outMax;
	else if(ITerm < outMin) ITerm= outMin;
	float dInput = (input - lastInput);

	/*Compute PID Output*/
	Output = kp * error + ITerm- kd * dInput;
	if(Output > outMax) Output = outMax;
	else if(Output < outMin) Output = outMin;

	/*cout << "P " << (kp * error) << endl;
	cout << "I " << ITerm << endl;
	cout << "D " << (kd * dInput) << endl;*/

	/*Remember some variables for next time*/
	lastInput = input;
	
	return Output;
}

void cPID::SetTunings(float Kp, float Ki, float Kd)
{
	if (Kp<0 || Ki<0|| Kd<0) return;

	kp = Kp;
	ki = Ki * SampleTime;
	kd = Kd / SampleTime;

	if(controllerDirection == DIR_REVERSE)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}

void cPID::SetSampleTime(int hz)
{
	float NewSampleTime = 1.f / hz;
	if (NewSampleTime > 0)
	{
		float ratio  = (float)NewSampleTime
				      / (float)SampleTime;
		ki *= ratio;
		kd /= ratio;
		SampleTime = (unsigned long)NewSampleTime;
	}
}

void cPID::SetOutputLimits(float Min, float Max)
{
	if(Min > Max) return;
	outMin = Min;
	outMax = Max;

	if(Output > outMax) Output = outMax;
	else if(Output < outMin) Output = outMin;

	if(ITerm > outMax) ITerm= outMax;
	else if(ITerm < outMin) ITerm= outMin;
}

void cPID::SetControllerDirection( enum Direction dir)
{
	// If we require inverting,
	if( dir != controllerDirection)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}

	// Always set dir
	controllerDirection = dir;
}
