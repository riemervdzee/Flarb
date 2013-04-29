#ifndef STRUCT_INPUTSTRING_H
#define STRUCT_INPUTSTRING_H

#include <vector>

/*
 * Enumerates the possible directions for the segments
 */
enum DIRECTION {
	DIR_LEFT,
	DIR_RIGHT,
	DIR_RETURN, // In assignment advanced we can return into the current row
};


/*
 * Declares a segment 
 */
struct sSegment {
	enum DIRECTION  rowdir;
	int             row_count;
};


/*
 * Manages the segments, plus it tells us where we are now
 */
struct sInputString {
	std::vector<sSegment> segments;  // List of all Segments
	int   currentSegment;            // The index of the current Segment we are in
};


#endif // STRUCT_INPUTSTRING_H
