/*
 * cCanbus.cpp - Send packages on a Lawicel canusb
 *
 *  Created on: 19-March-2013
 *      Author: Riemer van der Zee
 */

#include <iostream>
#include <string>
#include <stdio.h>     // standard input / output functions
#include <string.h>    // string function definitions
#include <unistd.h>    // UNIX standard function definitions
#include <fcntl.h>     // File control definitions
#include <errno.h>     // Error number definitions
#include <termios.h>   // POSIX terminal control definitions
#include <time.h>      // time calls
#include <stdlib.h>    //
#include <algorithm>   // std::max
using namespace std;

#include "flarb_canbus/cCanbus.h"
#include "flarb_canbus/cRosCom.h"

// The driver doesn't support packages with timestamps.
// Setting this 1, tells the canusb to put it off when opening the canbus connection
// This should be done atleast once (it is stored in the eeprom)
#define TURN_OFF_TIMESTAMP 1


/*
 * Opens a canbus connection
 */
int cCanbus::PortOpen( const char* device, int baudrate, int canSpeed, cRosCom* roscom, bool printSuccess)
{
	// Vars
	int ret;
	string SpeedChar = "";

	// set RosCom object
	_roscom = roscom;

	// Check if 'canSpeed' is in the range of 0-8
	if( canSpeed < 0 || canSpeed > 8) {
		ret = EINVAL;
		goto portopen_ret;
	}

	// Opens the serial port
	// The cSerial class outputs nice enough error messages, no need for doing it twice
	try {
		_serial.open( device, baudrate);
	}catch(...){}

	if( !_serial.isOpen())
	{
		ret = EINVAL;
		goto portopen_ret;
	}

	// Set timeout for reads
	_serial.setTimeout( boost::posix_time::milliseconds(5));

	// Clears the buffer of the just opened canbus
	ret = ClearModemCache();
	if(ret != 0) {
		printf( "Failed to clear modem buffer. device %s, error %i: %s\n", device, ret, strerror(ret));
		goto portopen_close;
	}

#if TURN_OFF_TIMESTAMP
	// Turn off the timestamps for incomming packages
	_serial.writeString( "Z0\r");
#endif

	// Set canbus speed
	SpeedChar.push_back( (char)(canSpeed + '0')); // promote int to ascii char
	_serial.writeString( "S" + SpeedChar + "\r");

	// Open the canbus connection to every other can device
	_serial.writeString( "O\r");

	// Get version
	/*buff[0] = 'V';
	buff[1] = CAN_DELIM;

	ret = SendCommand( buff, 2, 5, 5);
	if(ret != 0) {
		printf( "Failed to get the version. device %s, error: %i: %s\n", device, ret, strerror(ret));
		goto portopen_closecan;
	}

	// Copy version from buffer
	memcpy( _devVersion, _canbus_readbuffer, 5);
	_devVersion[5] = '\0';

	// Get serial
	buff[0] = 'N';
	buff[1] = CAN_DELIM;

	ret = SendCommand( buff, 2, 5 );
	if(ret != 0) {
		printf( "Failed to get the serial number. device %s, error: %i: %s\n", device, ret, strerror(ret));
		goto portopen_closecan;
	}

	// Copy serial number from buffer
	memcpy( _devSerial, _canbus_readbuffer, 5);
	_devSerial[5] = '\0';*/

	// Everything is set up, print something nice
	if( printSuccess) {
		printf(
				"Lawicel canbus is opened! Hurray \n" \
				"Port     : %s \n" \
//				"Version  : %s \n" 
//				"Serial   : %s \n" 
				"Baudrate : %i (internal representation)\n" \
				"Canspeed : S%i \n\n",
			device, /*_devVersion, _devSerial,*/ baudrate, canSpeed);
	}

	// We succeeded
	return 0;


	// Error unroling
portopen_closecan:
	_serial.writeString( "C\r");

portopen_close:
	_serial.close();

portopen_ret:
	return ret;
}

/*
 * Closes the canbus connection
 */
int cCanbus::PortClose()
{
	// Close the canbus, and close the serial
	_serial.writeString( "C\r");

	// Close the serialPort
	_serial.close();

	return 0;
}

/*
 * Clears the Lawicel modem buffers
 */
int cCanbus::ClearModemCache()
{
	// Send /r 3 times
	for( int i = 0; i < 3; i++)
		_serial.writeString( "\r");

	return 0;
}

/*
 * Process all packages on the bus
 */
int cCanbus::PortRead()
{
	// msg
	CanMessage msg;

	// try
	try {
		// Try to get a string
		string str = _serial.readStringUntil( "\r");
		if(str[0] == 'T')
		{
			// Get length, length is always in the range of 0-8
			// Get ID
			string id  = str.substr(1, 8);
			string len = str.substr(9, 1);

			// Get them to ints
			msg.identifier = strtoul(  id.c_str(), NULL, 16);
			msg.length     = strtoul( len.c_str(), NULL, 16);

			for(unsigned int i = 0; i < msg.length; i++)
			{
				string d = str.substr(10 + i * 2, 2);
				msg.data[i] = (char)strtoul( d.c_str(), NULL, 16);
			}

			// Process the msg
			_roscom->MessageReceived( msg);

// DEBUG
#if 0
			printf( "Received package. ID=%05X, length=%i \ndata: 0x", msg.identifier, msg.length);
			for( unsigned int i = 0; i < msg.length; i++)
			printf( "%02X", msg.data[i]);
#endif

		}
		// Ignore
		else if(str[0] == 'Z')
		{}
		else
		{
			cout << str << endl;
		}

	}catch(timeout_exception e){}

	return 0;
}

/*
 * Sends the message given
 */
int cCanbus::PortSend( const CanMessage &msg)
{
	// Our message buffer + pointer
	char buffer[27];
	char* buff = buffer;

	// Fill first part
	sprintf( buffer, "T%08X%X", msg.identifier, msg.length);
	buff += 10;

	// Fill the data
	for(unsigned int i = 0; i < msg.length; i++, buff+=2)
		sprintf( buff, "%02X", msg.data[i] );

	// Debug
#if 0
	printf( "Sending %s + \\r, length %i \n", buffer, int(buff - buffer+1));
#endif
	
	// Add delim byte
	sprintf( buff, "\r" );
	buff++;

	// Send message
	_serial.write( buffer, (int)(buff - buffer));
	return 0;
}

/*
 * Check the canbus errors
 *
 * returns 0 if no error, otherwise the correct flag(s) are set
 */
int cCanbus::CheckErrors()
{
	// Get status flags
	/*char buff[2];
	buff[0] = 'F';
	buff[1] = CAN_DELIM;
	SendCommand( buff, 2, 3, 3);

	// Get bit flag from the buffer
	_canbus_readbuffer[3] = '\0';
	int flags = strtoul( _canbus_readbuffer + 1, NULL, 16);

	// DEBUG
	//printf( "%i\n", flags);

	// Check flags (counting from bit 0)
	if( flags & (1 << 0)) printf( "Canbus error: CAN receive FIFO queue full\n");
	if( flags & (1 << 1)) printf( "Canbus error: CAN transmit FIFO queue full\n");
	if( flags & (1 << 2)) printf( "Canbus error: Error warning (EI), see SJA1000 datasheet\n");
	if( flags & (1 << 3)) printf( "Data Overrun (DOI), see SJA1000 datasheet\n");
	// bit 4 unused
	if( flags & (1 << 5)) printf( "Canbus error: Error Passive (EPI), see SJA1000 datasheet\n");
	if( flags & (1 << 6)) printf( "Canbus error: Arbitration Lost (ALI), see SJA1000 datasheet\n");
	if( flags & (1 << 7)) printf( "Canbus error: Bus Error (BEI), see SJA1000 datasheet\n");

	// Returns flags
	return flags;*/
	// TODO implement?
	return 0;
}

