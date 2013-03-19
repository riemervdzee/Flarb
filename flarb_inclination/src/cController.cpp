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

// Settings
#define DEV_PORT		"/dev/ttyS1" 	//port 
#define BAUD_RATE		9600			//Baudrate port
int open_port(void);
using namespace std;

// Functions executed at the beginning and end of the Node
bool cController::Create()
{
	// Topic name / buffer
	_rosTopic = _rosNode.advertise<std_msgs::String>( "inclino_updates", 100);
	datapackage[22];
	open_port();

	return true;
}

// Executed when the Node is exiting
void cController::Destroy()
{
	//close(fd);
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
		cout << "Try to open /dev/ttyS1"<< endl;
		fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
		if (fd == -1)
		{
	   		//Could not open the port.
			perror("open_port: Unable to open /dev/ttyS1 - ");
			//Try again
			sleep(1);
			open_port();
		}
		else
			fcntl(fd, F_SETFL, 0);
			return (fd);
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
	*		X=+01.123\r\nY=-12.123\r\n
	*
	*/
bool cController::getChar() {
			int counter = 0;
						
			char Temp[1024];
			// If the port is actually open, grab a String
			if (fd != -1) {
				cout << "FD! -1"<< endl;
				// Return the byte received if it's there
				int n = read(fd, &Temp, 1);
				
				for (int i = counter; i < counter + n; i++){
					datapackage[i] = Temp[i];
					//package is 22Bytes
					if(i == 21){
						//Convert X Axis
						int cntx = 0;
						char x[5];
						for (int j = 2; j < 9; j++){
							x[cntx]=datapackage[j];
							cntx++;						
						}
						xaxis = ::atof(x);
						//Convert Y Axis
						int cnty = 0;
						char y[5];
						for (int j = 13; j < 20; j++){
							y[cnty]=datapackage[j];
							cnty++;						
						}						
						yaxis = ::atof(y);
//						perror(yaxis , xaxis);
						cout << "test of het werkt"<< endl;
						cout << yaxis;
						cout << xaxis;					   					
					}					
				}
				counter += n;
				
			}
return true;
			
}

void reverseWord(char word [], char reverse [], int howMany)
{
	for (int i = howMany -1,j=0; i>=0; i--,j++)
	reverse[j] = word[i];
	return;
}
