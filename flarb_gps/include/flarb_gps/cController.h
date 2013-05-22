#ifndef CLASS_CONTROLLER_H
#define CLASS_CONTROLLER_H

#include "ros/ros.h"
#include "flarb_gps/cSerial.h"
#include "flarb_gps/GGA.h"
#include "flarb_gps/VTG.h"
#include "flarb_gps/RMC.h"
/*
* Main controller class of the example node
*/
class cController
{
public:
    // Functions executed at the beginning and end of the Application
    int Create();
    void Destroy();
	
    // Updates the Node
    void Update();

private:
	// Serial object
	cSerial _serial;
	char buff[512];
	int size;
	// Reference to the ros node handle
	ros::NodeHandle _rosNode;

	// We are publishing shizzle
	ros::Publisher _GGA;
	ros::Publisher _RMC;
	flarb_gps::GGA gga;
	flarb_gps::RMC rmc;	

	//Open port
	int Openport();
	//Read device
	int readDevice(int start);
	int getPackage(char* dataPack);
	//Data packs
	int GGAMessage(char *data, int counter);
	int RMCMessage(char *data, int counter);
	//checksum
	int checksum(char *s);
	//Opening FileDescriptor
	int OpenDevice();
	// Get package from the serial port
	int getPackage();

	int getData();
	char Buffer[1024];
	int ptrb;
};

#endif // CLASS_CONTROLLER_H
