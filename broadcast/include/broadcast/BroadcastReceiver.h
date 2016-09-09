#pragma once

#include <map>
#include <ros/ros.h>
#include <argus_utils/utils/LinalgTypes.h>
#include <argus_utils/synchronization/SynchronizationTypes.h>

#include "argus_msgs/FloatVectorStamped.h"
#include "broadcast/BroadcastInfoManager.h"
#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

/*! \brief Wraps looking up, receiving, and caching a data broadcast. */
class BroadcastReceiver
{
public:

	BroadcastReceiver();

	void Initialize( const std::string& streamName,
	                 const YAML::Node& props );

	void InitializePullStream( const std::string& streamName,
	                           const std::string& topic,
	                           QueryMode mode );

	void InitializePushStream( const std::string& streamName,
	                           const std::string& topic,
	                           QueryMode mode,
	                           double cacheTime = 5.0,
	                           unsigned int incomingQueueSize = 0 );

	void SetCacheTime( double time );

	unsigned int GetDim() const;
	const std::string& GetStreamName() const;
	
	bool IsReady() const;

	bool ReadStream( const ros::Time& time, StampedFeatures& features ) const;

	ros::Time EarliestTime() const;
	ros::Time LatestTime() const;

private:

	mutable Mutex _mutex;

	ros::NodeHandle _nodeHandle;
	LookupInterface _lookup;
	BroadcastInfoManager _infoManager;
	
	bool _initialized;
	std::string _streamName;
	QueryMode _queryMode;

	ros::Subscriber _pushSub;
	mutable ros::ServiceClient _pullClient;

	typedef std::map<ros::Time,StampedFeatures> StreamCache;
	StreamCache _featureCache;
	double _maxTimespan;

	bool PullStream( const ros::Time& time, StampedFeatures& f ) const;
	bool ReadCached( const ros::Time& time, StampedFeatures& f ) const;

	void FeatureCallback( const argus_msgs::FloatVectorStamped::ConstPtr& msg );
	void CheckTimespan();
	double GetTimespan() const;
};

}