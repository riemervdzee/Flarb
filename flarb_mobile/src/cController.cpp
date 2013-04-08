#include <iostream>
#include <sstream>
#include <sys/types.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
 
#include "flarb_mobile/cServer.h"
#include "flarb_mobile/cController.h"
#include "flarb_mobile/commands.pb.h"
#include "flarb_inclination/Axis.h" 
#include "sensor_msgs/LaserScan.h"

int port = 1337;
using namespace std;

/*
 *	Callback receiving message from MEAS Inclino
 */
void cController::axismsg(const flarb_inclination::AxisConstPtr& msgr)
{
	message_axis = *msgr;
	cout<< message_axis.x <<endl; 
}

/*
 *	Callback receiving message from LMS111 SICK Laser scanner
 */
void cController::LMSmsg(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	lms = *scan;
	//cout << lms.angle_min << endl;
	//the LaserScan message
	cout << "Header: "<<lms.header.stamp << endl;
	cout << "FrameID: "<<lms.header.frame_id << endl;
	cout << "Angle min: " << lms.angle_min << endl; 
	cout << "Angle max: " << lms.angle_max << endl;
	cout << "Angle Increment" << lms.angle_increment << endl;
	cout << lms.time_increment << endl;
	cout << lms.range_min << endl;
	cout << lms.range_max << endl;


}
 

/*
 *	Functions executed at the beginning and end of the Node
 */
bool cController::Create()
{
	//Creating Sockets
	//int sockfd, newsockfd, portno, clilen, n;

	//Listener of a message Arg1: message Arg2: buffer of messages Arg3: method called when package arrive 	
	_ax = _rosNode.subscribe<flarb_inclination::Axis>("Inclino_Axis", 1, &cController::axismsg, this);
	_LMS = _rosNode.subscribe<sensor_msgs::LaserScan>("scan", 5, &cController::LMSmsg, this);
	//TODO implement more callbacks so we can send data to our mobile device
	
	//TODO: if failed try again function for createSocket	
	//Creating server Accepting connections on port 1337
	//_server.createSocket(1337);	
	ros::spin();
	return true;
}

/*
 *	Executed when the Node is exiting
 */
void cController::Destroy()
{
	_server.closeSocket();
	ros::shutdown();
}

/*
 *	Updates the controller obj
 */
void cController::Update()
{	
	char buffer[256];
    bzero(buffer,256);
	if(_server.readData(buffer) == 0){
		//TODO method reading out packages	
	} 	
}

/*
 *	Send data over socket
 */
void cController::SendData()
{
	//Header a = 32;
	//SerializeTo0Stream
	//_server.sendData()
}

/*
 *	Read Package from clients
 * 	execute if it is an command
 */
void cController::packageReadout()
{
	
}





