#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"
#include "flarb_VDMixer/cController.h"
#include "flarb_inclination/Axis.h"
#include "flarb_compass/Compass.h"
#include "flarb_motorcontrol/Encoder.h"
#include "flarb_accelerometer/Accelerometer.h"
#include "flarb_gps/GGA.h"
#include "flarb_gps/RMC.h"

using namespace std;

/*
* Main controller class of the example node
*/
class cController
{
public:

    // Functions executed at the beginning and end of the Application
    bool Create();
    void Destroy();

    // Updates the Node
    void Update();

private:
	
	// Reference to the ros node handle
	ros::NodeHandle _rosNode;
	//Actual Subscriber
	ros::Subscriber Compass;
	//ros::Subscriber Gyro;
	ros::Subscriber Inclination;
	ros::Subscriber PhaseControl;
	ros::Subscriber GGA;
	ros::Subscriber RMC;
	
	
	flarb_inclination::Axis message_axis;
	flarb_compass::Compass message_north_angle;
	flarb_motorcontrol::Encoder message_phase;
	flarb_gps::GGA message_gga;
	flarb_gps::RMC message_rmc;

	void axismsg(const flarb_inclination::Axis msgr);
	void compassmsg(const flarb_compass::Compass msgr);
	void phasemsg(const flarb_motorcontrol::Encoder msgr);
	void GGAmsg(const flarb_gps::GGA msgr);
	void RMCmsg(const flarb_gps::RMC msgr);


	float WGS_KML(float Coord);
	int Log_KML(flarb_gps::GGA msg);
};

#endif // CLASS_CONTROLLER_H
