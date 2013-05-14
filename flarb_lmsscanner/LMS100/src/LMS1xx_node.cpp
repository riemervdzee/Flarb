#include <csignal>
#include <cstdio>
#include <LMS1xx.h>
#include "ros/ros.h"
#include "LMS100/LMSConfig.h"
#include "sensor_msgs/LaserScan.h"
#include "filters/filters.h"
using namespace std;

// Used in program
#define DEG2RAD M_PI/180.0

// Extra options
#define SEND_INTENSITIES 0


// Options setable via ROS service
bool EnableRawOutput = false;
int  FilterSelect = 2; // Median filter is standard


/**
 * Config service handler
 */
bool ConfigCallback( LMS100::LMSConfig::Request  &req, LMS100::LMSConfig::Response &res)
{
	// Check for out-of-range
	if(req.FilterSelect < 0 && req.FilterSelect > 3)
		return false;

	// Set data and return true
	EnableRawOutput = req.EnableRawOutput;
	FilterSelect    = req.FilterSelect;

	return true;
}


/**
 * Main entry of program
 */
int main(int argc, char **argv)
{
	// laser data
	LMS1xx laser;
	scanCfg cfg;
	scanDataCfg dataCfg;
	scanData data;

	// published data
	sensor_msgs::LaserScan scan_msg;

	// parameters
	string host;
	string frame_id;

	// ROS
	ros::init(argc, argv, "lms1xx");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/sick/scan", 1);
	ros::Publisher scanfil_pub = nh.advertise<sensor_msgs::LaserScan>("/sick/scan_filtered", 1);
	
	ros::ServiceServer LMSConfig = nh.advertiseService( "/sick/config", ConfigCallback);

	n.param<string>("host", host, "192.168.1.2");
	n.param<string>("frame_id", frame_id, "laser");

	// initialize hardware
	ROS_INFO("connecting to laser at : %s", host.c_str());	
	while (!laser.isConnected())
		laser.connect(host);

	while(!laser.isLoggedin())
		laser.login();

	ROS_INFO("Connected to laser and logged in.");


	cfg = laser.getScanCfg();

	scan_msg.header.frame_id = frame_id;

	scan_msg.range_min = 0.01;
	scan_msg.range_max = 20.0;

	scan_msg.scan_time = 100.0/cfg.scaningFrequency;

	scan_msg.angle_increment = (double)cfg.angleResolution/10000.0 * DEG2RAD;
	scan_msg.angle_min = (double)cfg.startAngle/10000.0 * DEG2RAD;
	scan_msg.angle_max = (double)cfg.stopAngle/10000.0 * DEG2RAD;

	cout << "resolution : " << (double)cfg.angleResolution/10000.0 << " deg " << endl;
	cout << "frequency : " << (double)cfg.scaningFrequency/100.0 << " Hz " << endl;

	int num_values;
	if (cfg.angleResolution == 2500)
	{
		num_values = 1081;
	}
	else if (cfg.angleResolution == 5000)
	{
		num_values = 541;
	}
	else
	{
		ROS_ERROR("Unsupported resolution");
		return 0;
	}
	
	// Init filters
	InitFilterNone    ( num_values);
	InitFilterAverage ( num_values);
	InitFilterMedian  ( num_values);
	InitFilterKalman  ( num_values);

	// Set some scan_msg defaults
	scan_msg.time_increment = scan_msg.scan_time/num_values;
	scan_msg.ranges.resize(num_values);

#if SEND_INTENSITIES
	scan_msg.intensities.resize(num_values);
	dataCfg.remission = true;
#else
	dataCfg.remission = false;
#endif

	dataCfg.outputChannel = 1;
	dataCfg.resolution = 1;
	dataCfg.encoder = 0;
	dataCfg.position = false;
	dataCfg.deviceName = false;
	dataCfg.outputInterval = 1;

	laser.setScanDataCfg(dataCfg);

	laser.startMeas();

	status_t stat;
	do // wait for ready status
	{
		stat = laser.queryStatus();
		ros::Duration(1.0).sleep();
	}
	while (stat != ready_for_measurement);

	laser.startDevice(); // Log out to properly re-enable system after config
	laser.scanContinous(1);


	while (ros::ok())
	{
		ros::Time start = ros::Time::now();

		scan_msg.header.stamp = start;
		++scan_msg.header.seq;

		laser.getData(data);

#if SEND_INTENSITIES
		for (int i = 0; i < data.rssi_len1; i++)
		{
			scan_msg.intensities[i] = data.rssi1[i];
		}
#endif

		// Send to /sick/scan if raw data is wanted
		if( EnableRawOutput)
		{
			for (int i = 0; i < data.dist_len1; i++)
			{
				scan_msg.ranges[i] = data.dist1[i] * 0.001;
			}

			scan_pub.publish(scan_msg);
		}
		
		switch( FilterSelect)
		{
			case 1:
				ExecuteFilterAverage( data, scan_msg);
				break;

			case 2:
				ExecuteFilterMedian( data, scan_msg);
				break;

			// TODO wait till implemented
			/*case 3:
				ExecuteFilterKalman( data, scan_msg);
				break;*/

			default:
				ExecuteFilterNone( data, scan_msg);
				break;
		}
		
		scanfil_pub.publish(scan_msg);

		ros::spinOnce();
	}
	
	// de-init filters
	DestroyFilterNone();
	DestroyFilterAverage();
	DestroyFilterMedian();
	DestroyFilterKalman();

	laser.scanContinous(0);
	laser.stopMeas();
	laser.disconnect();

  return 0;
}
