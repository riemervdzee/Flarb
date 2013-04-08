#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"
#include "flarb_VDMixer/cController.h"

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
	//ros::Subscriber Compass;
	//ros::Subscriber Accelerator;
	//ros::Subscriber Gyro;
	ros::Subscriber Inclination;
	//ros::Subscriber PhaseControl;

	flarb_inclination::Axis message_axis;


	int _count;
	
};

#endif // CLASS_CONTROLLER_H
