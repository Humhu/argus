#include "broadcast/BroadcastCommon.h"
#include "argus_utils/utils/MatrixUtils.h"

namespace argus
{

std::string BroadcastModeToString( BroadcastMode mode )
{
	switch( mode )
	{
		case PUSH_TOPIC:
			return "push";
		case PULL_TOPIC:
			return "pull";
		default:
			throw std::runtime_error( "BroadcastModeToString: Unknown mode." );
	}
}

BroadcastMode StringToBroadcastMode( const std::string& str )
{
	if( str == "push" )
	{
		return PUSH_TOPIC;
	}
	else if( str == "pull" )
	{
		return PULL_TOPIC;
	}
	else
	{
		throw std::runtime_error( "StringToBroadcastMode: Unknown string: " + str );
	}
}

std::string QueryModeToString( QueryMode mode )
{
	switch( mode )
	{
		case CLOSEST_BEFORE:
			return "closest_before";
		case CLOSEST_AFTER:
			return "closest_after";
		case CLOSEST_EITHER:
			return "closest_either";
		default:
			throw std::runtime_error( "QueryModeToString: Unknown mode." );
	}
}

QueryMode StringToQueryMode( const std::string& str )
{
	if( str == "closest_before" )
	{
		return CLOSEST_BEFORE;
	}
	else if( str == "closest_after" )
	{
		return CLOSEST_AFTER;
	}
	else if( str == "closest_either" )
	{
		return CLOSEST_EITHER;
	}
	else
	{
		throw std::runtime_error( "StringToQueryMode: Unknown string: " + str );
	}
}

StampedFeatures::StampedFeatures() {}

StampedFeatures::StampedFeatures( const ros::Time& t, const std::string& n, 
                                  const VectorType& v )
: time( t ), name( n ), features( v ) {}


StampedFeatures::StampedFeatures( const argus_msgs::FloatVectorStamped& msg )
: time( msg.header.stamp ), 
  name( msg.header.frame_id ),
  features( GetVectorView( msg.values ) ) {}



argus_msgs::FloatVectorStamped StampedFeatures::ToMsg() const
{
	argus_msgs::FloatVectorStamped msg;
	msg.header.stamp = time;
	msg.header.frame_id = name;
	SerializeMatrix( features, msg.values );
	return msg;
}

}