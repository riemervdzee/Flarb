#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include <ros/ros.h>
#include "flarb_simulation/cRosCom.h"
#include "flarb_simulation/cMap.h"
#include "flarb_simulation/cCar.h"

/*
* Main controller class of the example node
*/
class cController
{
public:
    // Functions executed at the beginning and end of the Application
    bool Create( int hz);
    void Destroy();

    // Updates the Node
    void Update();

private:

	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// RosCom handle
	cRosCom _roscom;

	// Basic handles
	cMap _map;
	cCar _car;
};

#endif // CLASS_CONTROLLER_H
