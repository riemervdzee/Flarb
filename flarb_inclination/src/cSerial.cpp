/*
 * cSerial.cpp - Send stuff to a serial (non-modem) port
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

#include "flarb_inclination/cSerial.h"

/*
 * Opens a serial non-modem 'device' with the given baudrate
 * Returns 0 if success, errno if failed
 */
int cSerial::PortOpen( const char* device, int baudrate)
{
	// Our ending return value
	int ret = 0;
	
	// Open the file
	int fd_flags = O_RDWR | O_NOCTTY /*| O_NDELAY*/ | O_SYNC;
	_fileDescriptor = open( device, fd_flags);

	if( _fileDescriptor < 0)
	{
		printf( "Failed to open device %s, error %i: %s\n", device, errno, strerror(errno));
		ret = errno;
		goto portopen_return;
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
		ret = errno;
		goto portopen_close;
	}
	
	// Success
	return 0;
	
	// Error unrolling
portopen_close:
	close( _fileDescriptor);
	_fileDescriptor = -1;

portopen_return:
	return ret;
}

/*
 * Closes the serial port
 * Returns 0 for success, errno for errors
 */
int cSerial::PortClose()
{
	// Close the file
	if( close( _fileDescriptor) == -1)
		return errno;
	
	// Reset fileDescriptor and return success
	_fileDescriptor = -1;
	return 0;
}

/*
 * Reads the incomming data from the port to the buffer
 * returns amount of bytes read. returns 0 if no data available.
 * returns -1 if there is an error, error is stored in errno
 */
int cSerial::Read( char* buffer, int maxlength)
{
	//
	int n = read( _fileDescriptor, buffer, maxlength);
	
	// If there was no data available, just cheat and say there are 0 bytes
	if( n == -1 && errno & (EAGAIN | EWOULDBLOCK ))
		n = 0;

	// Return amount of bytes written to 'buffer', or any other error value
	return n;
}

/*
 * Writes 'buffer' with 'length' to the serial port.
 * Returns amount of bytes written, -1 if failed (error is in errno)
 */
int cSerial::Write( const char* buffer, int length)
{
	// Writes data to the file
	return write( _fileDescriptor, buffer, length);
}


/*
 * Reads a fixed amount of incoming data from the port to the buffer,
 * with a certain amount of retries
 *
 * returns 0 for success, errno if error(s) occured
 */
int cSerial::ReadBytes( char* buffer, int amount, int retries)
{
	// Vars
	int tries = 0, bytes_read = 0, end = amount;

	// Retries
	for( ; end > 0; tries++)
	{
		// Are we above the retries?
		if( tries == retries)
			return EIO;

		// Attempt to read bytes
		int ret = Read( buffer + bytes_read, end);

		// If we actually read some bytes, add it to bytes_read
		if(ret > 0)
		{
			bytes_read += ret;
			end        -= ret;
		}
		// If error condition
		else if( ret == -1)
		{
			return errno;
		}
	}

	// We survived the unslaught above, we succeeded yay!
	return 0;
}

