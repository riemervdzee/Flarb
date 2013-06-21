#ifndef _FILTERS_H_
#define _FILTERS_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <LMS1xx.h>

/*
 * Which filter to select
 *    0 = None
 *    1 = Averaging
 *    2 = Median filter
 */

// Inits & deinits
void InitFilterNone    ( const int size);
void InitFilterAverage ( const int size);
void InitFilterMedian  ( const int size);

void DestroyFilterNone();
void DestroyFilterAverage();
void DestroyFilterMedian();

// Filters actually doing something
void ExecuteFilterNone    ( const scanData &data, sensor_msgs::LaserScan &msg);
void ExecuteFilterAverage ( const scanData &data, sensor_msgs::LaserScan &msg);
void ExecuteFilterMedian  ( const scanData &data, sensor_msgs::LaserScan &msg);

#endif
