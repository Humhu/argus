#include "broadcast/BroadcastReceiver.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/MapUtils.hpp"

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
	if( !_infoManager.CheckMemberInfo( streamName, true, ros::Duration( 5.0 ) ) )
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

VectorType BroadcastReceiver::GetClosestReceived( const ros::Time& time ) const
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