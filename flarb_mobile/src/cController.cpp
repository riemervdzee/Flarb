#include <iostream>
#include <sstream>
#include <sys/types.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include "flarb_mobile/cController.h"
#include "flarb_mobile/commands.pb.h"
#include "flarb_inclination/Axis.h" //Reading

using namespace std;

void cController::axismsg(const flarb_inclination::AxisConstPtr& msgr)
{
	message_axis = *msgr;
	cout<< message_axis.x <<endl; 
	//TODO: Package Size
	//TODO: Package	Actual
}

// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	//Creating Sockets
	//int sockfd, newsockfd, portno, clilen, n;

	// Topic name / buffer
	//_data = _rosNode.advertise<flarb_mobile::Axis>("Mobile_data", 100);

	//Listener of a message Arg1: message Arg2: buffer of messages Arg3: method called when package arrive 	
	//flarb_inclination::Axis msgmessages;	
	//_ax = _rosNode.subscribe("Inclino_Axis", 100, Axismsg); 
	//flarb_inclination::Axis it(_rosNode);
	_ax = _rosNode.subscribe<flarb_inclination::Axis>("Inclino_Axis", 1, &cController::axismsg, this);
	//Creating server Accepting connections on port 1337
	
	ros::spin();
	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	//now shutdown ros
	ros::shutdown();
}

// Updates the controller obj
void cController::Update()
{
	//TODO:Update
}



void cController::SendData()
{
	
}

