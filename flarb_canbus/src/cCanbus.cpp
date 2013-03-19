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

// Our great buffer
#define BUF_SIZE     128
char buf[ BUF_SIZE];

#include "flarb_canbus/cCanbus.h"

#define CAN_DELIM '\r'   // Modem delimitor

// Opens a canbus connection
int cCanbus::PortOpen( const char* device, int baudrate, int canSpeed) 
{
	// Vars
	int ret;
	char buff[3];

	// Opens the serial port
	// The cSerial class outputs nice enough error messages, no need for doing it twice
	ret = _serial.PortOpen( device, baudrate);
	if( ret != 0)
		return ret;

	// Clears the buffer of the just opened canbus
	ret = ClearBuff();
	if(ret != 0) {
		printf( "Failed to clear modem buffer. device %s, error %i: %s\n", device, ret, strerror(ret));
		return ret;
	}

	// Set canbus speed
	buff[0] = 'S';
	buff[1] = canSpeed + '0';
	buff[2] = CAN_DELIM;

	ret = SendCommand( buff, 3 );
	if(ret != 0) {
		printf( "Failed to set canbus speed. device %s, error %i: %s\n", device, ret, strerror(ret) );
		return ret;
	}

	// Open the canbus connection to every other can device
	buff[0] = 'O';
	buff[1] = CAN_DELIM;

	ret = SendCommand( buff, 2 );
	if(ret != 0) {
		printf( "Failed to open canbus. device %s, error: %i: %s\n", device, ret, strerror(ret));
		return ret;
	}
    
    // Get version and serial
    // Open the canbus connection to every other can device
    // TODO get version and serial#
	/*buff[0] = 'V';
	buff[1] = CAN_DELIM;

	SendCommand( buff, 2 );*/
	
	// We succeeded
	return 0;
	
	// TODO unroll errors
}

// Closes the canbus connection
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

// 1  = package read
// 0  = no packages available
// <0 = error
// msg* is a pointer where the PortRead function can put the package in
// Keep calling this func till it returns 0

/* TODO implement properly */
int cCanbus::PortRead( CanMessage* msg)
{
	int n = _serial.Read( buf, BUF_SIZE);

	// If no bytes are read, return
	if( n == 0)
		return 0;

	printf( "Read bytes, %i, value= 0x", n);
	
	for(int i = 0; i < n; i++)
		printf( "%02X", *(buf+i));
		
	printf( "\n");
	
	return 0;
}

// Sends the message mentioned
int cCanbus::PortSend( CanMessage* msg) { return 0;}

/*
 * Clears the modem buffers
 */
int cCanbus::ClearBuff()
{
	char buff[1] = { CAN_DELIM};
	int ret = 0;

	// Send /r 3 times
	for(int i = 0; ret == 0 && i < 3; i++)
		ret = SendCommand( buff, 1);

	return ret;
}

/* TODO implement these, duh */
int cCanbus::CheckErrors() { return 0;}
int cCanbus::GetVersion()  { return 0;}
int cCanbus::GetSerial()   { return 0;}

// Helper function
int cCanbus::SendCommand( const char* string, int length, int pos)
{
	// Vars
	int ret;

	// Check if our pos ain't out of range
	if( pos >= BUF_SIZE)
		return EINVAL;

	// Write the command
	ret = _serial.Write( string, length);
	if( ret != length)
		return ret;

	// Check the status (13 = OK, 7 = fail)
	int n = 0;
	while( n == 0) //TODO don't spin indefinitly, check for errors
		n = _serial.Read( buf, pos + 1);

	// get the specific char
	char status = buf[ pos];

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

