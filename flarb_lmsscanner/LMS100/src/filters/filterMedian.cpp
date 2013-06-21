#include "filters.h"

#define MEDIAN_SIZE 3

static int *buffer   = NULL;
static int  position = 0;
static bool filled   = false;


void InitFilterMedian ( const int size) 
{
	int s = MEDIAN_SIZE;
	buffer = new int[s * size];
}

void DestroyFilterMedian () 
{
	delete buffer;
	buffer   = NULL;
	position = 0;
	filled   = false;
}

void ExecuteFilterMedian ( const scanData &data, sensor_msgs::LaserScan &msg)
{
	// Vars
	int offset_start = 0;
	int offset_pos   = position;
	int median       = (MEDIAN_SIZE / 2);
	std::vector<int> temp ( MEDIAN_SIZE);

	for (int i = 0; i < data.dist_len1; i++)
	{
		buffer[offset_pos]  = data.dist1[i];

		if(filled)
		{
			for(int j = 0; j < MEDIAN_SIZE; j++)
			{
				temp[j] = buffer[offset_start + j];
			}
			std::sort (temp.begin(), temp.begin()+MEDIAN_SIZE); 

			msg.ranges[i] = (float)temp[median]  * 0.001f;
		}
		else
			msg.ranges[i] = data.dist1[i] * 0.001;

		// Increase offsets
		offset_start += (MEDIAN_SIZE);
		offset_pos   += (MEDIAN_SIZE);
	}

	position++;

	if( position == MEDIAN_SIZE)
	{
		position = 0;
		filled = true;
	}
}
