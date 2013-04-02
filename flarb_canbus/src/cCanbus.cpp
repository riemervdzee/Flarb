/*
 * cCanbus.cpp - Send stuff to canusb
 *
 *  Created on: 19-March-2013
 *      Author: Riemer van der Zee
 */

#include <stdio.h>     // standard input / output functions
#include <string.h>    // string function definitions
#include <unistd.h>    // UNIX standard function definitions
#include <fcntl.h>     // File control definitions
#include <errno.h>     // Error number definitions
#include <termios.h>   // POSIX terminal control definitions
#include <time.h>      // time calls
#include <stdlib.h>    //
#include <algorithm>   // std::max

#include "flarb_canbus/cCanbus.h"

// Modem delimitor
#define CAN_DELIM '\r' // Carriage return  0xB
#define CAN_ERROR '\a' // Bell character   0x7


/*
 * Constructor/deconstructor, pretty boring but they get the buffer alrighty
 */
cCanbus::cCanbus()  { _canbus_readbuffer = new char[CANBUS_READBUFFER_SIZE]; }
cCanbus::~cCanbus() { delete _canbus_readbuffer;}

/*
 * Opens a canbus connection
 */
int cCanbus::PortOpen( const char* device, int baudrate, int canSpeed, cRosCom* roscom)
{
	// Vars
	int ret;
	char buff[3];

	// set RosCom object
	_roscom = roscom;

	// Opens the serial port
	// The cSerial class outputs nice enough error messages, no need for doing it twice
	ret = _serial.PortOpen( device, baudrate);
	if( ret != 0)
		goto portopen_ret;

	// Clears the buffer of the just opened canbus
	ret = ClearModemCache();
	if(ret != 0) {
		printf( "Failed to clear modem buffer. device %s, error %i: %s\n", device, ret, strerror(ret));
		goto portopen_close;
	}

	// Set canbus speed
	buff[0] = 'S';
	buff[1] = canSpeed + '0';
	buff[2] = CAN_DELIM;

	ret = SendCommand( buff, 3 );
	if(ret != 0) {
		printf( "Failed to set canbus speed. device %s, error %i: %s\n", device, ret, strerror(ret) );
		goto portopen_close;
	}

	// Open the canbus connection to every other can device
	buff[0] = 'O';
	buff[1] = CAN_DELIM;

	ret = SendCommand( buff, 2 );
	if(ret != 0) {
		printf( "Failed to open canbus. device %s, error: %i: %s\n", device, ret, strerror(ret));
		goto portopen_close;
	}

	// Get version
	buff[0] = 'V';
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
	_devSerial[5] = '\0';

	// Everything is set up, print something nice
	printf(
			"Lawicel canbus is opened! Hurray \n" \
			"Port     : %s \n" \
			"Version  : %s \n" \
			"Serial   : %s \n" \
			"Baudrate : %i (internal representation)\n" \
			"Canspeed : S%i \n\n",
		device, _devVersion, _devSerial, baudrate, canSpeed);

	// We succeeded
	return 0;


	// Error unroling
portopen_closecan:
	buff[0] = 'C';
	buff[1] = CAN_DELIM;

	SendCommand( buff, 2 );

portopen_close:
	_serial.PortClose();

portopen_ret:
	return ret;
}

/*
 * Closes the canbus connection
 */
int cCanbus::PortClose()
{
	// Close canbus connection
	char buff[2];
	buff[0] = 'C';
	buff[1] = CAN_DELIM;

	// Don't care about the return value, we are leaving ship anyway
	SendCommand( buff, 2 );

	// Close the serialPort
	_serial.PortClose();

	return 0;
}

/*
 * Clears the Lawicel modem buffers
 */
int cCanbus::ClearModemCache()
{
	char buff[1] = { CAN_DELIM};
	int ret = 0;

	// Send /r 3 times. Note we can receive both CAN_DELIM and CAN_ERROR
	for(int i = 0; (ret == 0 || ret == ENOEXEC) && i < 3; i++)
		ret = SendCommand( buff, 1, 0, 0);

	return ret;
}

/*
 * Process all packages on the bus
 */
int cCanbus::PortRead()
{
	int ret;

	// Keep on reading packages, till it fails
	while ( (ret = ReadPackage( 10, false)) == 0) {}

	// TODO look if it is a valid 'error'

	return 0;
}

/*
 * Sends the message given
 */
int cCanbus::PortSend( const CanMessage &msg)
{
	// Our message buffer + pointer
	char buffer[14];
	char* buff = buffer;

	// Fill first part
	sprintf( buffer, "t%03X%X", msg.identifier, msg.length);
	buff += 5;

	// Fill the data
	for(int i = 0; i < msg.length; i++, buff+=2)
		sprintf( buff, "%02X", msg.data[i] );

	// Debug
	printf( "Sending %s + \\r, length %i \n", buffer, int(buff - buffer+1));
	
	// Add delim byte
	sprintf( buff, "\r" );
	buff++;

	// Send message
	return SendCommand( buffer, (int)(buff - buffer), 1, 1);
}

/*
 * Check the canbus errors
 */
int cCanbus::CheckErrors()
{
	// Get status flags
	char buff[2];
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

	// TODO maybe return an error code as well.. not important
	return 0;
}

/*
 * Helper function for reads
 * 't' is already in the buffer if skipFirst is true
 *
 * Returns     0  Success, we read a package! (only one!)
 *            -1  If there are no packages
 *           EIO  Unknown package
 *         errno  other
 * 
 */
int cCanbus::ReadPackage( int retries, bool skipFirst)
{
	// Vars
	int ret;
	CanMessage msg;

	// Should we check if there is a package at all?
	if( skipFirst == false)
	{
		// Read one byte
		ret = _serial.ReadBytes( _canbus_readbuffer, 1, retries);

		// EIO is given when there were no bytes to read
		if( ret == EIO)
			return -1;
		else if( ret != 0) // For the remaining errors
		{
			printf( "ReadPackage: Could not read\n");
			return ret;
		}

		// Check if it is a Package, then handle it
		if( _canbus_readbuffer[0] != 't')
		{
			// Unknown package, abbandon ship! TODO clear buffers, everywhere
			printf( "ReadPackage: Unknown package! fu fu fu\n");
			return EIO;
		}
	}

	// If we got this far, we can assume 't' is in the _canbus_readbuffer. Now we must read the rest
	ret = _serial.ReadBytes( _canbus_readbuffer, 4, retries);
	if( ret != 0)
	{
		printf( "ReadPackage: Could not read\n");
		return ret;
	}

	// Get length, length is always in the range of 0-8
	msg.length = _canbus_readbuffer[3] - '0';

	// Get identifier
	_canbus_readbuffer[3] = '\0';
	msg.identifier = strtoul( _canbus_readbuffer, NULL, 16);

	// Get data
	ret = _serial.ReadBytes( _canbus_readbuffer, msg.length * 2 , retries);
	if( ret != 0)
	{
		printf( "ReadPackage: Could not read\n");
		return ret;
	}

	// Get the data from the buffer
	char databuffer[3] = { '\0'};
	for( int i = 0; i < msg.length; i++)
	{
		databuffer[0] = _canbus_readbuffer[ i * 2];
		databuffer[1] = _canbus_readbuffer[ i * 2 + 1];

		msg.data[i] = (char)strtoul( databuffer, NULL, 16);
	}

	// TODO enable, Process the msg
	//_roscom->PublishMessage( msg);

	// DEBUG
	printf( "Received package. ID=%03X, length=%i \ndata: 0x", msg.identifier, msg.length);
	for(int i = 0; i < msg.length; i++)
		printf( "%02X", msg.data[i]);

	printf( "\n");

	return 0;
}

/*
 * Helper function for writes
 */
int cCanbus::SendCommand( const char* string, int length, int charPositionRight, int charPositionFalse)
{
	// Vars
	int ret, pos, nBytes;
	const int retries = 10;

	// Get max
	pos = std::max( charPositionRight, charPositionFalse);

	// Check if our pos ain't out of range
	if( pos >= CANBUS_READBUFFER_SIZE){
		printf( "SendCommand: Buffer is too small, aborting\n");
		return EINVAL;
	}

	// Write the command
	ret = _serial.Write( string, length);
	if( ret != length)
		return ret;

	// Get the amount of bytes we need to read
	// normally we would add +1, but since we are already reading one byte in advance..
	nBytes = std::max( charPositionRight, charPositionFalse);

	// Should we check for a package?
	bool checkForPackage = true;

	// Loop till there are no packages left
	while( checkForPackage)
	{
		// Read one byte
		ret = _serial.ReadBytes( _canbus_readbuffer, 1, retries);
		if( ret != 0)
		{
			printf( "ReadCommand: Could not read\n");
			return ret;
		}

		// Check if it is a Package, then handle it
		if( _canbus_readbuffer[0] == 't')
		{
			// Handle the received package
			ret = ReadPackage( retries, true);
			if( ret != 0)
				return ret;
		}
		else
			checkForPackage = false;
	}

	// TODO check for charPositionFalse first (read to charPositionFalse). Not really important
	// Get extra bytes if neccessary
	if( nBytes != 0)
	{
		// Note, skip the first readbuffer byte here
		ret = _serial.ReadBytes( _canbus_readbuffer + 1, nBytes, retries);
		if( ret != 0)
		{
			printf( "ReadCommand: Read failed\n");
			return ret;
		}
	}

	// Check the positions
	if      ( _canbus_readbuffer[ charPositionFalse] == CAN_ERROR)
		return ENOEXEC;
	else if ( _canbus_readbuffer[ charPositionRight] == CAN_DELIM)
		return 0;
	else
	{
		// DEBUG
		//for( int i = 0; i < nBytes; i++)
		//	printf( "%02X", _canbus_readbuffer[i] );
		//printf( " = data. charFalse %i, charRight %i\n", charPositionFalse, charPositionRight);

		// Print error
		printf( "ReadCommand: wrong position, oh noes\n");
		return EINVAL;
	}

	return 0;
}

