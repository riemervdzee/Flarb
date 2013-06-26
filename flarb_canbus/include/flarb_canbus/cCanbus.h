#ifndef CLASS_CANBUS_H
#define CLASS_CANBUS_H

#include "flarb_canbus/cSerial.h"

// Class prototypes
class cRosCom;


/*
 * Canbus message struct
 */
struct CanMessage {
	unsigned int identifier;   /* In the range of 000-7FF hex */
	unsigned int length;	   /* In the range of 0-8 hex */
	unsigned char data[8];	   /* Per byte 0-255 */
};


/*
 * cCanbus.h - Send packages on a Lawicel canusb
 */
class cCanbus
{
public:

	// Functions executed at the beginning and end of the Application
	int PortOpen( const char* device, int baudrate, int canSpeed, cRosCom* roscom, bool printSuccess);
	int PortClose();

	// Reads the port for messages, sends them to RosCom object
	int PortRead();

	// Sends the message mentioned
	int PortSend( const CanMessage &msg);

	// Clears the Lawicel modem buffers
	int ClearModemCache();

	// Check for canbus errors
	int CheckErrors();

	// Get Lawicel version/serial number
	inline const char* GetVersion() { return _devVersion;}
	inline const char* GetSerial()  { return _devSerial;}

private:

	// Serial object
	cSerial _serial;

	// RosCom object __CANBUS IS NOT THE OWNER OF THIS OBJ__
	cRosCom* _roscom;

	// Serial and version number of the Lawicel usb2can device
	char _devSerial [6];
	char _devVersion[6];
};

#endif // CLASS_CANBUS_H

