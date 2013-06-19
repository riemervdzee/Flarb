#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"
#include "flarb_VDMixer/cController.h"

#include "flarb_msgs/State.h"
#include "flarb_msgs/Axis.h"
#include "flarb_msgs/Compass.h"
#include "flarb_msgs/Encoder.h"
#include "flarb_msgs/GGA.h"
#include "flarb_msgs/RMC.h"
using namespace std;

/*
 * Main controller class of the example node
 */
class cController
{
public:
	// C-tor
	cController() : distance(0.0f) {}

    // Functions executed at the beginning and end of the Application
    bool Create();
    void Destroy();

    // Updates the Node
    void Update();

	// State callback
	bool StateCallback( flarb_msgs::State::Request &req, flarb_msgs::State::Response &res);

private:
	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// Service handle
	ros::ServiceServer StateService;

	// Our subscribers
	ros::Subscriber Inclino;
	ros::Subscriber Compass;
	ros::Subscriber Encoder;
	ros::Subscriber GGA;
	ros::Subscriber RMC;

	// Saved messages
	flarb_msgs::Axis    message_axis;
	flarb_msgs::Compass message_compass;
	flarb_msgs::Encoder message_encoder;
	flarb_msgs::GGA     message_gga;
	flarb_msgs::RMC     message_rmc;

	//
	float distance;

	// Callbacks
	void axismsg    (const flarb_msgs::Axis msgr);
	void compassmsg (const flarb_msgs::Compass msgr);
	void encodermsg (const flarb_msgs::Encoder msgr);
	void GGAmsg     (const flarb_msgs::GGA msgr);
	void RMCmsg     (const flarb_msgs::RMC msgr);
};

#endif // CLASS_CONTROLLER_H
