#ifndef CLASS_CANBUS_H
#define CLASS_CANBUS_H

#include "flarb_canbus/cSerial.h"

/*
 * Canbus message struct
 * TODO padding faults?
 */
struct CanMessage {
	unsigned int  identifier; /* In the range of 000-7FF */
	unsigned char length;	 /* In the range of 0-8 */
	unsigned char data[8];	/* Per byte 0-255 */
};

/*
 * Canbus class
 */
class cCanbus
{
public:
	// Constructor
	cCanbus() {}

	// Functions executed at the beginning and end of the Application
	int PortOpen( const char* device, int baudrate, int canSpeed);
	int PortClose();

	// Check for packages TODO determine arguments (vector?)
	// 1  = package read
	// 0  = no packages available
	// <0 = error
	// msg* is a pointer where the PortRead function can put the package in
	// Keep calling this func till it returns 0
	int PortRead( CanMessage* msg);

	// Sends the message mentioned
	int PortSend( CanMessage* msg);

	// Util functions
	int ClearBuff();
	int CheckErrors();
	int GetVersion();
	int GetSerial();

//private:
	// Helper function
	int SendCommand( const char* string, int length, int pos = 0);

private:
	// Serial object
	cSerial _serial;
};

#endif // CLASS_CANBUS_H

