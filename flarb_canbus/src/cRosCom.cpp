#include <algorithm>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "flarb_canbus/cRosCom.h"
using namespace std;

/*
 * Basically the init-er of cRosCom
 */
int cRosCom::Create( ros::NodeHandle *rosNode, cCanbus *canbus)
{
	// Set canbus ref
	_canbus  = canbus;
	_rosNode = rosNode;

	// Subscribe to /canbus/send
	_canSend = _rosNode->subscribe<flarb_canbus::CanMessage>( "/canbus/send", 100, &cRosCom::SendCallback, this);

	// Setup service
	_srvSubscribe = _rosNode->advertiseService
					( "canbus/subscribe", &cRosCom::SubscribeCallback, this);

	// TODO add error paths
	return 0;
}

/*
 * Stuff is quite boring here, ain't it?
 */
int cRosCom::Destroy()
{
	return 0;
}

/**
 * Puts a message on the Canbus
 */
void cRosCom::SendCallback( const flarb_canbus::CanMessageConstPtr &msg)
{
	// Construct a canbus message
	struct CanMessage canmessage;
	canmessage.identifier	= msg->identifier;
	canmessage.length		= msg->data.size();

	// Copy data
	for( int i = 0; i < canmessage.length; i++)
		canmessage.data[i] = msg->data.at( i);

	// Forward message to canbus
	_canbus->PortSend( canmessage);
}

/**
 * Helper function for below
 */
static bool vectorSort( const sRosComPublishEntry i, const sRosComPublishEntry j)
{
	return ( i.id < j.id);
}

/**
 * We have a new Subscriber!
 */
bool cRosCom::SubscribeCallback( flarb_canbus::CanSubscribe::Request  &req,
                                 flarb_canbus::CanSubscribe::Response &res)
{
	//TODO Check for uniqueness
	// Set first stuff
	sRosComPublishEntry entr;
	entr.id = req.identifier;

	// Create topic entry
	entr.topic = _rosNode->advertise<flarb_canbus::CanMessage>( req.topicname, 100);

	// Add entry to the vector
	_PublishEntries.push_back( entr);

	// Sort on ID
	// TODO check if this going alrighty
	std::sort( _PublishEntries.begin(), _PublishEntries.end(), vectorSort );

	// TODO add error paths
	return true;
}

/**
 * We received a message on the canbus, publish if we got the right subscriber
 */
void cRosCom::PublishMessage( const struct CanMessage &canmessage)
{
	// Vars
	vector< sRosComPublishEntry>::iterator itr;
	flarb_canbus::CanMessage msg;

	// Loop through all entries, to get the right one
	for ( itr = _PublishEntries.begin(); itr != _PublishEntries.end(); itr++)
	{
		if( itr->id == canmessage.identifier)
			break;
	}

	// Check if we haven't found anything at all
	if( itr == _PublishEntries.end())
	{
		// TODO return big ass error message
		return;
	}

	// Construct the msg
	msg.identifier = canmessage.identifier;
	for( int i = 0; i < canmessage.length; i++)
		msg.data.push_back( canmessage.data[i]);

	// Send the message
	itr->topic.publish( msg);
}

