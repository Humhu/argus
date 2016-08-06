#pragma once

#include <ros/ros.h>

#include "broadcast/BroadcastCommon.h"
#include "broadcast/BroadcastInfoManager.h"
#include "broadcast/QueryFeatures.h"

#include "argus_utils/utils/LinalgTypes.h"

#include <map>

namespace argus
{

/*! \brief Wraps registering and publishing a feature broadcast. */
class BroadcastTransmitter
{
public:

	BroadcastTransmitter();

	void InitializePushStream( const std::string& streamName, 
	                           ros::NodeHandle& nh,
	                           unsigned int featureSize,
	                           const std::vector<std::string> descriptions,
	                           unsigned int outgoingQueueSize = 10,
	                           const std::string& topic = "stream_raw" );

	void InitializePullStream( const std::string& streamName, 
	                           ros::NodeHandle& nh,
	                           unsigned int featureSize,
	                           const std::vector<std::string> descriptions,
	                           double cacheTime = 1.0,
	                           const std::string& topic = "pull_stream" );

	void Publish( const ros::Time& stamp, const VectorType& feats );

private:

	ros::NodeHandle _nodeHandle;
	LookupInterface _lookup;
	BroadcastInfoManager _infoManager;

	std::string _streamName;

	ros::Publisher _streamPub;
	ros::ServiceServer _streamServer;

	typedef std::map<ros::Time, StampedFeatures> StreamCache;
	StreamCache _cache;
	double _cacheTime;

	bool QueryCallback( broadcast::QueryFeatures::Request& req,
	                    broadcast::QueryFeatures::Response& res );

	void CacheStream( const StampedFeatures& f );
	double CacheSpan() const;
};

}