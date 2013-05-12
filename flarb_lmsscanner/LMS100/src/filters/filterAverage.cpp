#include "filters.h"

#define AVERAGE_SIZE 5

static int *buffer   = NULL;
static int  position = 0;
static bool filled   = false;


void InitFilterAverage ( const int size) 
{
	int s = AVERAGE_SIZE + 1;
	buffer = new int[s * size];
}

void DestroyFilterAverage () 
{
	delete buffer;
	buffer   = NULL;
	position = 0;
	filled   = false;
}

void ExecuteFilterAverage ( const scanData &data, sensor_msgs::LaserScan &msg)
{
	// Offsets
	int offset_sum = 0;
	int offset_pos = position;

	for (int i = 0; i < data.dist_len1; i++)
	{
		buffer[offset_sum] += data.dist1[i] - buffer[offset_pos];
		buffer[offset_pos]  = data.dist1[i];

		if(filled)
		{
			msg.ranges[i] = (float)(buffer[offset_sum] / AVERAGE_SIZE)  * 0.001f;
		}
		else
			msg.ranges[i] = data.dist1[i] * 0.001;

		// Increase offsets
		offset_sum += (AVERAGE_SIZE+1);
		offset_pos += (AVERAGE_SIZE+1);
	}

	position++;

	if( position == AVERAGE_SIZE)
	{
		position = 0;
		filled = true;
	}
}
