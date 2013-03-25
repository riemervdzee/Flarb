#ifndef CLASS_CANBUS_H
#define CLASS_CANBUS_H

#include "flarb_canbus/cSerial.h"


#define CANBUS_READBUFFER_SIZE    128


/*
 * Canbus message struct
 */
struct CanMessage {
	unsigned short identifier; /* In the range of 000-7FF hex */
	unsigned short length;	   /* In the range of 0-8 hex */
	unsigned char data[8];	   /* Per byte 0-255 */
};


/*
 * Canbus class
 */
class cCanbus
{
public:
	// Constructor and deconstructor
	cCanbus();
	~cCanbus();

	// Functions executed at the beginning and end of the Application
	int PortOpen( const char* device, int baudrate, int canSpeed);
	int PortClose();

	// 1  = package read
	// 0  = no packages available
	// <0 = error
	// msg* is a pointer where the PortRead function can put the package in
	// Keep calling this func till it returns 0
	int PortRead( CanMessage* msg);

	// Sends the message mentioned
	int PortSend( const CanMessage* msg);

	// Clears the Lawicel modem buffers
	int ClearModemCache();

	// Check for canbus errors
	int CheckErrors();

	// Get Lawicel version/serial number
	inline const char* GetVersion() { return _devVersion;}
	inline const char* GetSerial()  { return _devSerial;}

private:
	// Helper functions
	int SendCommand( const char* string, int length, int charPositionRight = 0, int charPositionFalse = 0);
	int ReadCommand( int charPositionRight, int charPositionFalse, int retries);
	int ReadPackage( int retries, bool skipFirst = false);

	// Forbit copy constructor
	cCanbus( const cCanbus&);

	// Serial object
	cSerial _serial;

	// Read buffer
	char *_canbus_readbuffer;

	// Serial and version number of the Lawicel usb2can device
	char _devSerial [6];
	char _devVersion[6];
};

#endif // CLASS_CANBUS_H

