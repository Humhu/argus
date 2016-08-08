#include "broadcast/BroadcastTransmitter.h"
#include "argus_msgs/FloatVectorStamped.h"

#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/utils/MapUtils.hpp"

#include "lookup/LookupUtils.hpp"

using namespace broadcast;

namespace argus
{

BroadcastTransmitter::BroadcastTransmitter() 
: _lookup(), _infoManager( _lookup ) {}

void BroadcastTransmitter::InitializePushStream( const std::string& streamName, 
	                                             ros::NodeHandle& nh,
                                                 unsigned int featureSize,
                                                 const std::vector<std::string> descriptions,
                                                 unsigned int outgoingQueueSize,
                                                 const std::string& topic )
{
	_streamName = streamName;
	register_lookup_target( nh, streamName );

	BroadcastInfo info;
	info.featureSize = featureSize;
	info.featureDescriptions = descriptions;
	info.topic = nh.resolveName( topic );
	info.mode = PUSH_TOPIC;
	_infoManager.WriteMemberInfo( streamName, info );

	_streamPub = nh.advertise<argus_msgs::FloatVectorStamped>( topic, outgoingQueueSize );
}

void BroadcastTransmitter::InitializePullStream( const std::string& streamName, 
	                                             ros::NodeHandle& nh,
                                                 unsigned int featureSize,
                                                 const std::vector<std::string> descriptions,
                                                 double cacheTime,
                                                 const std::string& topic )
{
	_streamName = streamName;
	_cacheTime = cacheTime;
	register_lookup_target( nh, streamName );

	BroadcastInfo info;
	info.featureSize = featureSize;
	info.featureDescriptions = descriptions;
	info.topic = nh.resolveName( topic );
	info.mode = PULL_TOPIC;
	_infoManager.WriteMemberInfo( streamName, info );

	_streamServer = nh.advertiseService( topic, &BroadcastTransmitter::QueryCallback, this );
}

void BroadcastTransmitter::Publish( const ros::Time& stamp, const VectorType& features )
{
	const BroadcastInfo& info = _infoManager.GetInfo( _streamName );
	if( features.size() != info.featureSize )
	{
		throw std::runtime_error( "Received features of size " + std::to_string( features.size() )
		                          + " for stream " + _streamName + " but expected " 
		                          + std::to_string( info.featureSize ) );
	}

	BroadcastMode mode = _infoManager.GetInfo( _streamName ).mode;
	StampedFeatures f( stamp, _streamName, features );
	switch( mode )
	{
		case PUSH_TOPIC:
		{
			_streamPub.publish( f.ToMsg() );
			break;
		}
		case PULL_TOPIC:
		{
			CacheStream( f );
			break;
		}
		default:
		{
			throw std::runtime_error( "BroadcastTransmitter: Unknown broadcast mode." );
		}
	}
}

bool BroadcastTransmitter::QueryCallback( QueryFeatures::Request& req,
                                          QueryFeatures::Response& res )
{
	BroadcastMode mode = _infoManager.GetInfo( _streamName ).mode;
	if( mode != PULL_TOPIC )
	{
		throw std::runtime_error( "BroadcastTransmitter::QueryCallback called on non-pull topic!" );
	}

	StreamCache::const_iterator iter;
	switch( req.time_mode )
	{
		case CLOSEST_BEFORE:
		{
			if( !get_closest_lesser_eq( _cache, req.query_time, iter ) ) { return false; }
			break;
		}
		case CLOSEST_AFTER:
		{
			if( !get_closest_greater_eq( _cache, req.query_time, iter ) ) { return false; }
			break;

		}
		case CLOSEST_EITHER:
		{
			if( !get_closest( _cache, req.query_time, iter ) ) { return false; }
			break;
		}
		default:
		{
			throw std::runtime_error( "BroadcastTransmitter: Unknown time query mode." );
		}
	}
	res.features = iter->second.ToMsg();
	return true;
}

void BroadcastTransmitter::CacheStream( const StampedFeatures& f )
{
	_cache[f.time] = f;
	while( CacheSpan() > _cacheTime )
	{
		_cache.erase( _cache.begin() );
	}
}

double BroadcastTransmitter::CacheSpan() const
{
	if( _cache.empty() ) { return 0; }
	return ( get_highest_key( _cache ) - get_lowest_key( _cache ) ).toSec();
}

}