#include "broadcast/BroadcastReceiver.h"
#include "broadcast/QueryFeatures.h"

#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/MapUtils.hpp"
#include "argus_utils/utils/ParamUtils.h"

#include <ros/service.h>
#include <sstream>

namespace argus
{

BroadcastReceiver::BroadcastReceiver() 
: _nodeHandle(), _lookup(), _infoManager( _lookup ), _initialized( false )
{}

void BroadcastReceiver::Initialize( const std::string& streamName,
                                    const YAML::Node& props )
{
	std::string queryStr;
	GetParamRequired( props, "query_mode", queryStr );
	QueryMode queryMode = StringToQueryMode( queryStr );

	if( !_infoManager.CheckMemberInfo( streamName, true, ros::Duration( 10.0 ) ) )
	{
		throw std::runtime_error( "Could not find broadcast stream: " + streamName );
	}

	const BroadcastInfo& info = _infoManager.GetInfo( streamName );
	switch( info.mode )
	{
		case PUSH_TOPIC:
		{
			double cacheTime;
			unsigned int queueSize;
			GetParam<double>( props, "cache_time", cacheTime, 5.0 );
			GetParam<unsigned int>( props, "queue_size", queueSize, 0 );
			InitializePushStream( streamName, info.topic, queryMode, cacheTime, queueSize );
			break;
		}
		case PULL_TOPIC:
		{
			InitializePullStream( streamName, info.topic, queryMode );
			break;
		}
		default:
		{
			throw std::runtime_error( "BroadcastReceiver: Unknown topic mode." );
		}
	}
}

void BroadcastReceiver::InitializePullStream( const std::string& streamName,
                                              const std::string& topic,
                                              QueryMode mode )
{
	_streamName = streamName;
	_queryMode = mode;
	ROS_INFO_STREAM( "Setting up to pull stream " << streamName << " on service: " << topic );
	ros::service::waitForService( topic );
	_initialized = true;
	_pullClient = _nodeHandle.serviceClient<broadcast::QueryFeatures>( topic, true );
}

void BroadcastReceiver::InitializePushStream( const std::string& streamName,
                                              const std::string& topic,
                                              QueryMode mode,
                                              double cacheTime,
                                              unsigned int queueSize )
{
	_streamName = streamName;
	_queryMode = mode;
	_maxTimespan = cacheTime;
		ROS_INFO_STREAM( "Listening to push stream " << streamName << " on topic: " << topic );
	_initialized = true;
	_pushSub = _nodeHandle.subscribe( topic, 
	                                  queueSize, 
	                                  &BroadcastReceiver::FeatureCallback, 
	                                  this );
}

void BroadcastReceiver::SetCacheTime( double cacheTime )
{
	WriteLock lock( _mutex );

	_maxTimespan = cacheTime;
}

unsigned int BroadcastReceiver::GetDim() const
{
	return _infoManager.GetInfo( _streamName ).featureSize;
}

const std::string& BroadcastReceiver::GetStreamName() const
{
	return _streamName;
}

bool BroadcastReceiver::IsReady() const
{
	ReadLock lock( _mutex );
	switch( _infoManager.GetInfo( _streamName ).mode )
	{
		case PUSH_TOPIC:
			return !_featureCache.empty();
		case PULL_TOPIC:
			return _initialized;
		default:
			throw std::runtime_error( "BroadcastReceiver: Unknown topic mode." );
	}
}

bool BroadcastReceiver::ReadStream( const ros::Time& time, StampedFeatures& f ) const
{
	ReadLock lock( _mutex );
	
	switch( _infoManager.GetInfo( _streamName ).mode )
	{
		case PUSH_TOPIC:
			return ReadCached( time, f );
		case PULL_TOPIC:
			return PullStream( time, f );
		default:
			throw std::runtime_error( "BroadcastReceiver: Unknown topic mode." );
	}
}

ros::Time BroadcastReceiver::EarliestTime() const
{
	return get_lowest_key( _featureCache );
}

ros::Time BroadcastReceiver::LatestTime() const
{
	return get_highest_key( _featureCache );
}

bool BroadcastReceiver::PullStream( const ros::Time& time, StampedFeatures& f ) const
{
	broadcast::QueryFeatures srv;
	srv.request.time_mode = _queryMode;
	srv.request.query_time = time;
	if( !_pullClient.call( srv ) )
	{
		return false;
	}
	f = StampedFeatures( srv.response.features );
	return true;
}

bool BroadcastReceiver::ReadCached( const ros::Time& time, StampedFeatures& f ) const
{
	StreamCache::const_iterator closest = _featureCache.end();

	switch( _queryMode )
	{
		case CLOSEST_BEFORE:
			if( !get_closest_lesser_eq( _featureCache, time, closest ) )
			{
				return false;
			}
			break;
		case CLOSEST_AFTER:
			if( !get_closest_greater_eq( _featureCache, time, closest ) )
			{
				return false;
			}
			break;
		case CLOSEST_EITHER:
			if( !get_closest_eq( _featureCache, time, closest ) )
			{
				return false;
			}
	}
	if( closest == _featureCache.end() )
	{
		return false;
	}
	f = closest->second;
	return true;
}

void BroadcastReceiver::FeatureCallback( const argus_msgs::FloatVectorStamped::ConstPtr& msg )
{
	WriteLock lock( _mutex );
	
	StampedFeatures f( *msg );
	if( f.name != _streamName ) 
	{ 
		ROS_WARN_STREAM( "Received features for " << f.name << " but expected "
		                 << _streamName );
		return; 
	}
	const BroadcastInfo& info = _infoManager.GetInfo( _streamName );
	if( f.features.size() != info.featureSize )
	{
		ROS_WARN_STREAM( "Feature stream: " << _streamName << " has dimension: "
		                 << f.features.size() << " but expected: " << info.featureSize );
	}
	_featureCache[msg->header.stamp] = f;
	CheckTimespan();
}

double BroadcastReceiver::GetTimespan() const
{
	if( _featureCache.empty() ) { return 0; }
	ros::Time latestTime = _featureCache.rbegin()->first;
	ros::Time earliestTime = _featureCache.begin()->first;
	return (latestTime - earliestTime).toSec();
}

void BroadcastReceiver::CheckTimespan()
{
	
	ros::Time latestTime = _featureCache.rbegin()->first;
	while( true )
	{
		ros::Time earliestTime = _featureCache.begin()->first;
		double span = (latestTime - earliestTime).toSec();
		if( span < _maxTimespan ) { break; }
		_featureCache.erase( _featureCache.begin() );
	}
}

}