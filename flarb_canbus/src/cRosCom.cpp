
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_canbus/cRosCom.h"

int cRosCom::Create( ros::NodeHandle &_rosNode, const cCanbus &canbus)
{
	// Subscribe to /canbus/send
	_canSend = _rosNode.subscribe<flarb_canbus::CanMessage>( "/canbus/send", 100, &cRosCom::SendCallback, this);

	//
	return 0;
}

int cRosCom::Destroy()
{
	return 0;
}

/**
 * Canbus f
 */
void cRosCom::SendCallback( const flarb_canbus::CanMessageConstPtr &msg)
{

}

/*void cRosCom::SubscribeCallback( const flarb_canbus::CanMessageConstPtr &msg)
{

}*/
