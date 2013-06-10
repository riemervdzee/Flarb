// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_VDMixer/cController.h"

// Functions executed at the beginning and end of the Node


/*
 *	Callback receiving message from MEAS Inclino
 */
void cController::axismsg(const flarb_inclination::Axis msgr)
{
	message_axis = msgr;
}

/*
 * 	Callback message Compass
 *	
 */
void cController::compassmsg(const flarb_compass::Compass msgr)
{
	message_north_angle = msgr;
}

/*
 *	Callback message PhaseControl (Must be encoder or some kind...)
 */
void cController::phasemsg(const flarb_motorcontrol::Encoder msgr)
{
	message_phase = msgr;
}

/*
 *	Callback message NMEA/GGA
 */
void cController::GGAmsg(const flarb_gps::GGA msgr)
{
	message_gga = msgr;
}

/*
 *	Callback message NMEA/RMC
 */
void cController::RMCmsg(const flarb_gps::RMC msgr)
{
	message_rmc = msgr;
}

bool cController::Create()
{
	//Subscriber for data 
	Compass = _rosNode.subscribe<flarb_compass::Compass>("sensor/compass", 1, &cController::compassmsg, this);
	GGA = _rosNode.subscribe<flarb_gps::GGA>("NMEA/GGA",1, &cController::GGAmsg, this);
	RMC = _rosNode.subscribe<flarb_gps::RMC>("NMEA/RMC",1, &cController::RMCmsg, this);
	//TODO: Modify inclination to standard
	Inclination = _rosNode.subscribe<flarb_inclination::Axis>("sensor/inclination", 1, &cController::axismsg, this);
	PhaseControl = _rosNode.subscribe<flarb_motorcontrol::Encoder>("steering/encoder", 1, &cController::phasemsg, this);
	return 0;
}

// Executed when the Node is exiting
void cController::Destroy()
{

}

// Updates the controller obj
void cController::Update()
{
	float k = 5250.7544; 
	WGS_KML(k);

	float l = 00501.9356; 
	WGS_KML(l);
}

int cController::Log_KML(flarb_gps::GGA msg){
	return 0;
}

float cController::WGS_KML(float Coord){
	setprecision(6);
	char data[32];
	char last[6];
	float KML = 0; 
	bool start = true;
	float aftherFloat = 0;
	memset(last, '-', 6);
	memset(data, '-', 32);
	//cout<< "data Coord original" << Coord << endl;
	//Getting point at the right position
	KML = Coord / 100.0;
	//Convert to Char
	sprintf(data,"%f",KML);
	//cout<< "integer: " << (int) KML << endl;
	//cout<< "KML: " << KML << endl;
	//cout<< "Coord: "<< Coord << endl;
	//cout<< "Data: "<< data << endl; 	
	int Size = sizeof(data);
	for (int i = 0; i < Size; i++){			
		if(data[i] == '.'){
			int counter = 0;
			for(int j = i + 1; j < i + 7; j++){
				last[counter] = data[j];			
				counter++;	
			}
			i = sizeof(data);
			aftherFloat = ::atof(last);
			cout<< "Atof raw: " << aftherFloat << endl;
			aftherFloat =  aftherFloat / 600000;
			cout<< "Aftherfloat2: " << aftherFloat << endl;
			int klm = 0; 
			klm = Coord / 100;			
			aftherFloat += (float) klm;
			cout<< "Aftherfloat4: " << aftherFloat << endl;
			
		}						
	}

	
	return Coord;
}

