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

#include "flarb_canbus/cCanbus.h"

#define CAN_DELIM '\r'   // Modem delimitor

// Opens a canbus connection
int cCanbus::PortOpen( const char* device, int baudrate, int canSpeed) 
{
	int ret = 0;
	// Open the file
	int fd_flags = O_RDWR | O_NOCTTY /*| O_NDELAY*/ | O_SYNC;
	_fileDescriptor = open( device, fd_flags);

	if( _fileDescriptor < 0)
	{
		printf( "Failed to open device %s, error %i: %s\n", device, errno, strerror(errno));
		return errno;
	}

	// Configure port
	struct termios port_settings;
	memset( &port_settings, 0, sizeof (port_settings));

	cfsetispeed( &port_settings, baudrate);    // set baud rates
	cfsetospeed( &port_settings, baudrate);
	
	port_settings.c_iflag &= ~IGNBRK;         // ignore break signal
	port_settings.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
	port_settings.c_oflag = 0;                // no remapping, no delays
	port_settings.c_cc[VMIN]  = 0;            // read doesn't block
	port_settings.c_cc[VTIME] = 1;            // 0.1 seconds read timeout

	port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	port_settings.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
	port_settings.c_cflag &= ~PARENB;  // Set no parity
	port_settings.c_cflag &= ~CSTOPB;  // Only 1 stopbit
	port_settings.c_cflag &= ~CSIZE;   // Clear CSize
	port_settings.c_cflag |= CS8;      // Set datasize to 8 bits

    // apply the settings to the port
	if( tcsetattr( _fileDescriptor, TCSANOW, &port_settings) != 0) {
		printf( "Failed to set serial parameters. device %s, error %i: %s\n", device, errno, strerror(errno));
		return errno;
	}

	// Clears the buffer of the just opened canbus
	ret = ClearBuff();
	if(ret != 0) {
		printf( "Failed to clear modem buffer. device %s, error %i: %s\n", device, ret, strerror(ret));
		return ret;
	}

	// Set canbus speed
	char buff[3];
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
	
	// Fill the filedescriptor set
	FD_ZERO( &_fileDescSet);
    FD_SET( _fileDescriptor, &_fileDescSet);
    
    // Get version and serial
    // Open the canbus connection to every other can device
	buff[0] = 'V';
	buff[1] = CAN_DELIM;

	SendCommand( buff, 2 );
	
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
	
	// Close the file
	close( _fileDescriptor);
	
	return 0;
}

// Checks the topic for messages to send, checks port for messages to
// Put on the topic back again
//int Update();

#define BUF_SIZE     32
unsigned char buf[ BUF_SIZE];

// Check for packages TODO determine arguments (vector?)
// 1  = package read
// 0  = no packages available
// <0 = error
// msg* is a pointer where the PortRead function can put the package in
// Keep calling this func till it returns 0

/* TODO implement properly */
int cCanbus::PortRead( CanMessage* msg)
{
	if( _fileDescriptor == -1)
		return -1;

	int n = read( _fileDescriptor, buf, BUF_SIZE);

	if( n == -1)
		return 0;

	printf( "Read bytes, %i, value=%s\n", n, buf);
	
	return 0;
}

// Sends the message mentioned
int cCanbus::PortSend( CanMessage* msg) { return 0;}

// Clears the modem buffers
int cCanbus::ClearBuff()
{
	char buff[3] = { CAN_DELIM};
	return SendCommand( buff, 3 );
}

/* TODO implement these, duh */
int cCanbus::CheckErrors() { return 0;}
int cCanbus::GetVersion()  { return 0;}
int cCanbus::GetSerial()   { return 0;}

// Helper function
int cCanbus::SendCommand( const char* string, int length)
{
	if( _fileDescriptor == -1)
		return -1;

	// Write the command
	if( write( _fileDescriptor, string, length ) != length)
		return errno;

	// TODO poll for read (13 = OK, 7 = fail)


	// Succes
	return 0;
}
