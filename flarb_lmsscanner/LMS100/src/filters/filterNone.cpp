#include "filters.h"

void InitFilterNone ( const int size) {}
void DestroyFilterNone() {}

void ExecuteFilterNone ( const scanData &data, sensor_msgs::LaserScan &msg)
{
	for (int i = 0; i < data.dist_len1; i++)
	{
		msg.ranges[i] = data.dist1[i] * 0.001;
	}
}
