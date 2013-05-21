#ifndef CLASS_PID_H
#define CLASS_PID_H

enum Direction {
	DIR_DIRECT,
	DIR_REVERSE,
};

class cPID
{
public:
	// C-tor
	cPID() {}
	cPID( int hz, enum Direction dir, float min, float max, float Kp, float Ki, float Kd);

	// Executes PID control
	float Execute( float goal, float input);

	// Set factors
	void SetTunings(float Kp, float Ki, float Kd);

	// Reset sample time
	void SetSampleTime(int hz);

	// Reset limits
	void SetOutputLimits(float Min, float Max);

	// Set direction
	void SetControllerDirection( enum Direction dir);

private:
	// The output given
	float Output;

	// PID factors
	float kp, ki, kd;

	// Temps
	float ITerm, lastInput;

	// = 1.f / hz. the timeslice in seconds
	float SampleTime;

	// Lower and upper bounds of the output
	float outMin, outMax;

	// Direction
	enum Direction controllerDirection;
};

#endif // CLASS_PID_H
