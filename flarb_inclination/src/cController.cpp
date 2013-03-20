// Include order: cppstd, ROS, Boost, own-module includes
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include "flarb_inclination/cController.h"
#include "flarb_inclination/cSerial.h"

// Settings
#define DEV_PORT		"/dev/ttyS1" 	//port 
#define BAUD_RATE		B9600			//Baudrate port
// Our great buffer
#define BUF_SIZE     128
char buf[ BUF_SIZE];

int open_port(void);
using namespace std;

// Functions executed at the beginning and end of the Node
int cController::Create()
{
	// Topic name / buffer
	_rosTopic = _rosNode.advertise<std_msgs::String>( "inclino_updates", 100);
	char Str[1024];
	int counter = 0;
	open_port();
	return 0;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	//close(fd);
	_serial.PortClose();
}

// Updates the controller obj
void cController::Update()
{
	// Increase count
	_count++;

	// Assemble message
	std_msgs::String msg;
	std::stringstream ss;
	//cout << "Hello World" << endl; 
	ss << "hello world " << _count;
	msg.data = ss.str();
	cout << "i am alive"<< endl;
	// Publish
	_rosTopic.publish(msg);
	getChar();
}

	/*
	* 'open_port()' - Open serial port 1.
	*
	* Returns the file descriptor on success or -1 on error.
	*
	* 9600 Baudrate, ASCII, 8 data bits, 1 stop bit, no parity
	*/
	int cController::open_port()
	{
		// Opens the serial port
		// The cSerial class outputs nice enough error messages, no need for doing it twice
		fd = _serial.PortOpen(DEV_PORT, BAUD_RATE);
		if( fd != 0){
			return fd;
		}
	}


	/*
	* This method will check if there is an packagage available.
	* if so it will return two floats X and Y 
	* 
	* layout of the package that's send over the RS232 Port
	* 22 Byte 
	* Layout: < D0 ... D21>
	* 		D0 ... D10	= “X=±xx.xxx“, <CR>, <LF> with D2 = sign (+ or -), D5 = decimal point
	* 		D11 ... D21 = “Y=±xx.xxx“, <CR>, <LF> with D13= sign (+ or -), D16 = decimal point
	*		Example: "X=+01.123\r\nY=-12.123\r\n"
	*		Return: 0 = Serial ok, -1 = Serial bad 
	*/
int cController::getChar() {
	char buffer[255];  /* Input buffer 				*/
	char *bufptr;      /* Current char in buffer 	*/
	int  nbytes;       /* Number of bytes read 		*/
	int  tries;        /* Number of tries so far 	*/

	for (tries = 0; tries < 3; tries ++)
	{
		/* read characters into our string buffer until we get a CR or NL */
		bufptr = buffer;
		while ((nbytes = read(fd, bufptr, buffer + sizeof(buffer) - bufptr - 1)) > 0)
		{
		bufptr += nbytes;
		if (bufptr[-1] == '\n' || bufptr[-2] == '\r')
			cout << "found one" <<  endl;
			break;
		}

		/* nul terminate the string and see if we got an OK response */
		*bufptr = '\0';
		printf(bufptr);
		cout << bufptr << endl;
		//if (strncmp(buffer, "X=", 2) == 0)
			
		//if (strncmp(buffer, "Y=", 2) == 0)
	}
	return (-1);	
			
}


