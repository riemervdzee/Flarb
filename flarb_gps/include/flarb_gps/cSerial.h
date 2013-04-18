/*
 * cSerial.cpp - Send stuff to a serial (non-modem) port
 *
 *  Created on: 19-March-2013
 *      Author: Riemer van der Zee
 */

#ifndef CLASS_SERIAL_H
#define CLASS_SERIAL_H


/*
 * Serial helper class
 */
class cSerial
{
public:
	/*
	 * Empty constructor, setting the file handle to a negative error code
	 */
	cSerial() : _fileDescriptor( -1) {}

	/*
	 * Opens a serial non-modem 'device' with the given baudrate
	 * Returns 0 if success, errno if failed
	 */
	int PortOpen( const char* device, int baudrate);
	
	/*
	 * Closes the serial port
	 * Returns 0 for success, errno for errors
	 */
	int PortClose();

	/*
	 * Reads the incomming data from the port to the buffer
	 * returns amount of bytes read. returns 0 if no data available.
	 * returns -1 if there is an error, error is stored in errno
	 */
	int Read( char* buffer, int maxlength);

	/*
	 * Reads a fixed amount of incoming data from the port to the buffer,
	 * with a certain amount of retries
	 *
	 * returns 0 for success, errno if error(s) occured
	 */
	int ReadBytes( char* buffer, int amount, int retries);

	/*
	 * Writes 'buffer' with 'length' to the serial port.
	 * Returns amount of bytes written, errno if failed
	 */
	int Write( const char* buffer, int length);

private:
	// File handle to the serial port
	int _fileDescriptor;
};

#endif // CLASS_SERIAL_H

