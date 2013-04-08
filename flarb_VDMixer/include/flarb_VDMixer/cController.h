#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"
#include "flarb_VDMixer/cController.h"
#include "flarb_inclination/Axis.h"
#include "flarb_compass/Compass.h"
#include "flarb_VDMixer/Phase.h"

using namespace std;

/*
* Main controller class of the example node
*/
class cController
{
public:
    // Constructor
    cController() : _count( 0) {}

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
	//ros::Subscriber Accelerator;
	//ros::Subscriber Gyro;
	ros::Subscriber Inclination;
	ros::Subscriber PhaseControl;

	flarb_inclination::Axis message_axis;
	flarb_compass::Compass message_north_angle;
	flarb_VDMixer::Phase message_phase;


	void axismsg(const flarb_inclination::Axis msgr);
	void compassmsg(const flarb_compass::Compass msgr);
	void phasemsg(const flarb_VDMixer::Phase msgr);
	int _count;
	
};

#endif // CLASS_CONTROLLER_H
