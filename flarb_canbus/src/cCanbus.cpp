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
#include <stdlib.h>

#include "flarb_canbus/cCanbus.h"

// Modem delimitor
#define CAN_DELIM '\r'


/*
 * Constructor/deconstructor, pretty boring but they get the buffer alrighty
 * TODO get rid of copy constructor. Not important
 */
cCanbus::cCanbus()  { _canbus_readbuffer = new char[CANBUS_READBUFFER_SIZE]; }
cCanbus::~cCanbus() { delete _canbus_readbuffer;}

/*
 * Opens a canbus connection
 */
int cCanbus::PortOpen( const char* device, int baudrate, int canSpeed) 
{
	// Vars
	int ret;
	char buff[3];

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

	ret = SendCommand( buff, 2, 5 );
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

	// Send /r 3 times
	for(int i = 0; ret == 0 && i < 3; i++)
		ret = SendCommand( buff, 1);

	return ret;
}

// 1  = package read
// 0  = no packages available
// <0 = error
// msg* is a pointer where the PortRead function can put the package in
// Keep calling this func till it returns 0
int cCanbus::PortRead( CanMessage* msg)
{
	int n = _serial.Read( _canbus_readbuffer, CANBUS_READBUFFER_SIZE);

	// If no bytes are read, return
	if( n == 0)
		return 0;

	printf( "Read bytes, error, %s %i, value= 0x", strerror(n), n);
	
	for(int i = 0; i < n; i++)
		printf( "%02X", *(_canbus_readbuffer + i) );
		
	printf( "\n");
	
	return 0;
}

/*
 * Sends the message given
 */
int cCanbus::PortSend( const CanMessage* msg)
{
	// Filter for null messages
	if (msg == NULL)
		return EINVAL;

	//
	char buffer[14];
	char* buff = buffer;

	// Fill first part
	sprintf( buffer, "t%03X%X", msg->identifier, msg->length);
	buff += 5;

	// Fill the data
	for(int i = 0; i < msg->length; i++, buff+=2)
		sprintf( buff, "%02X", msg->data[i] );
	
	// Add delim byte
	sprintf( buff, "\r" );
	buff++;

	// Debug
	printf( "Sending %s, length %i \n", buffer, buff - buffer);

	SendCommand( buffer, (int)(buff - buffer), 1);
	
	return 0;
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
	SendCommand( buff, 2, 3);

	// Get bit flag from the buffer
	_canbus_readbuffer[3] = '\0';
	int flags = strtoul( _canbus_readbuffer + 1, NULL, 16);
	
	//printf( "%i\n", flags);

	// Check flags (counting from bit 0)
	if( flags & (1 << 0)) printf( "Canbus error: CAN receive FIFO queue full\n");
	if( flags & (1 << 1)) printf( "Canbus error: CAN transmit FIFO queue full\n");
	if( flags & (1 << 2)) printf( "Canbus error: Error warning (EI), see SJA1000 datasheet\n");
	if( flags & (1 << 3)) printf( "Data Overrun (DOI), see SJA1000 datasheet\n");
	// bit 4 unused
	if( flags & (1 << 5)) printf( "Canbus error: Error Passive (EPI), see SJA1000 datasheet\n");
	if( flags & (1 << 5)) printf( "Canbus error: Arbitration Lost (ALI), see SJA1000 datasheet\n");
	if( flags & (1 << 5)) printf( "Canbus error: Bus Error (BEI), see SJA1000 datasheet\n");

	// TODO maybe return an error code as well.. not important
	return 0;
}

/*
 * Helper function for writes
 */
int cCanbus::SendCommand( const char* string, int length, int pos)
{
	// Vars
	int ret, i = 0, n = 0;

	// Check if our pos ain't out of range
	if( pos >= CANBUS_READBUFFER_SIZE)
		return EINVAL;

	// Write the command
	ret = _serial.Write( string, length);
	if( ret != length)
		return ret;

	// Check the status (13 = OK, 7 = fail), check it 10 times
	for( ; n == 0 && i < 10; i++)
		n = _serial.Read( _canbus_readbuffer, pos + 1);

	// We tried to read 10 times, no workies
	if( i == 10){
		printf( "SendCommand: Could not read\n");
		return EIO;
	}

	// TODO Error checking

	// get the specific char
	char status = _canbus_readbuffer[ pos];

	// Status stuff
	switch( status)
	{
		case 13:
			return 0;
		case 7:
			return EINVAL; // Input is invalid

		default:
			printf( "SendCommand: wrong position, oh noes\n");
			return EINVAL;
	}
}

