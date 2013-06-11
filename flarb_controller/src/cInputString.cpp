#include <sstream>
#include <iostream>
#include "flarb_controller/cInputString.h"
using namespace std;

/*
 * Example input: S - 3L - 0 - 2L - 2R - 1R - 5L - F
 */
cInputString::cInputString( const string &str)
{
	stringstream ss( str);
	string       buf;
	sSegment     seg;

	// Ignore the "S" and "-" part, check
	ss >> buf; if(buf[0] != 'S'){ cout << "Incorrect format! 1" << endl; return;}
	ss >> buf; if(buf[0] != '-'){ cout << "Incorrect format! 2" << endl; return;}

	// We break in the while loop later
	while(1)
	{
		// Check if there is still something in the stringstream
		if( ss.eof())
			{ cout << "Incorrect format! 3" << endl; return;}

		// Grab the next 
		ss >> buf;

		// Check if this is the end
		if( buf[0] == 'F')
			break;

		// Get the first number
		int number = buf[0] - '0';
		if( number < 0 || number > 9)
			{ cout << "Incorrect format! 4" << endl; return;}

		// 0 is special case, it should return in the current row
		if( number == 0)
		{
			seg.rowdir    = DIR_RETURN;
			seg.row_count = 0;
		}
		// 
		else
		{
			// Get dir
			if( buf[1] == 'L')
				seg.rowdir = DIR_LEFT;
			else if( buf[1] == 'R')
				seg.rowdir = DIR_RIGHT;
			else
				{ cout << "Incorrect format! 5" << endl; return;}

			// Put data
			seg.row_count = number;
		}

		// Push the segment found into the array
		segments.push_back( seg);

		// Ignore "-" part
		ss >> buf;
		if(buf[0] != '-'){ cout << "Incorrect format! 6" << endl; return;}
	}

	// DEBUG
#if 1
	for(int i = 0; i < segments.size(); i++)
	{
		sSegment seg = segments[i];
		cout << seg.row_count << " " << seg.rowdir << endl;
	}
#endif
}
