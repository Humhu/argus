#include "broadcast/BroadcastReceiver.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/MapUtils.hpp"

#include <sstream>

using namespace broadcast;

namespace argus
{

BroadcastReceiver::BroadcastReceiver( const std::string& streamName,
                                      double cacheTime,
                                      unsigned int incomingQueueSize,
                                      const std::string& topic )
: _nodeHandle(), _lookup(), _infoManager( _lookup ), 
_streamName( streamName ), _maxTimespan( cacheTime )
{
	if( !_infoManager.CheckMemberInfo( streamName, true, ros::Duration( 10.0 ) ) )
	{
		throw std::runtime_error( "Could not find broadcast stream: " + streamName );
	}
	const std::string& memberNamespace = _infoManager.GetNamespace( streamName );
	std::string streamTopic = memberNamespace + topic;
	ROS_INFO_STREAM( "Listening to stream on topic: " << streamTopic );
	_featSub = _nodeHandle.subscribe( streamTopic, 
	                                  incomingQueueSize, 
	                                  &BroadcastReceiver::FeatureCallback, 
	                                  this );
}

void BroadcastReceiver::SetCacheTime( double cacheTime )
{
	_maxTimespan = cacheTime;
}

const std::string& BroadcastReceiver::StreamName() const
{
	return _streamName;
}

bool BroadcastReceiver::HasReceived() const
{
	ReadLock lock( _mutex );
	return _featureMap.size() > 0;
}

bool BroadcastReceiver::Ready() const
{
	// return GetTimespan() >= _maxTimespan;
	return !_featureMap.empty();
}

VectorType BroadcastReceiver::GetClosestPrevious( const ros::Time& time ) const
{
	ReadLock lock( _mutex );

	MapType::const_iterator closest;
	if( !get_closest_lesser_eq( _featureMap, time, closest ) )
	{
		std::stringstream ss;
		ss << "No messages prior to query: " << time 
		   << ". Earliest time: " << get_lowest_key( _featureMap );
		// throw std::runtime_error( ss.str() );
		   ROS_WARN_STREAM( ss.str() );
		   return GetClosest( time );
	}
	if( closest == _featureMap.end() )
	{
		throw std::runtime_error( "Retrieval error." );
	}
	return closest->second;
}

VectorType BroadcastReceiver::GetClosest( const ros::Time& time ) const
{
	ReadLock lock( _mutex );

	MapType::const_iterator closest;
	if( !get_closest_eq( _featureMap, time, closest ) )
	{
		throw std::runtime_error( "Cannot retrieve messages from empty receiver." );
	}
	if( closest == _featureMap.end() )
	{
		throw std::runtime_error( "Retrieval error." );
	}
	return closest->second;
}

unsigned int BroadcastReceiver::OutputDim() const
{
	return _infoManager.GetInfo( _streamName ).featureSize;
}

void BroadcastReceiver::FeatureCallback( const StampedFeatures::ConstPtr& msg )
{
	WriteLock lock( _mutex );
	
	if( msg->header.frame_id != _streamName ) 
	{ 
		ROS_WARN_STREAM( "Received features for " << msg->header.frame_id << " but expected "
		                 << _streamName );
		return; 
	}
	VectorType features = GetVectorView( msg->features );
	const BroadcastInfo& info = _infoManager.GetInfo( _streamName );
	if( features.size() != info.featureSize )
	{
		ROS_WARN_STREAM( "Feature stream: " << _streamName << " has dimension: "
		                 << features.size() << " but expected: " << info.featureSize );
	}
	_featureMap[msg->header.stamp] = features;
	CheckTimespan();
}

double BroadcastReceiver::GetTimespan() const
{
	if( _featureMap.empty() ) { return 0; }
	ros::Time latestTime = _featureMap.rbegin()->first;
	ros::Time earliestTime = _featureMap.begin()->first;
	return (latestTime - earliestTime).toSec();
}

void BroadcastReceiver::CheckTimespan()
{
	
	ros::Time latestTime = _featureMap.rbegin()->first;
	while( true )
	{
		ros::Time earliestTime = _featureMap.begin()->first;
		double span = (latestTime - earliestTime).toSec();
		if( span < _maxTimespan ) { break; }
		_featureMap.erase( _featureMap.begin() );
	}
}

}