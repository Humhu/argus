#pragma once

#include <map>
#include <ros/ros.h>
#include <argus_utils/utils/LinalgTypes.h>
#include <argus_utils/synchronization/SynchronizationTypes.h>

#include "broadcast/StampedFeatures.h"
#include "broadcast/BroadcastInfoManager.h"

namespace argus
{

/*! \brief Wraps looking up, receiving, and caching a data broadcast. */
class BroadcastReceiver
{
public:

	BroadcastReceiver();

	void Initialize( const std::string& streamName,
	            ros::NodeHandle& ph );

	void Initialize( const std::string& streamName,
	                 QueryMode mode,
	                 double cacheTime = 5.0,
	                 unsigned int incomingQueueSize = 0 );

	void SetCacheTime( double time );

	unsigned int GetDim() const;
	const std::string& GetStreamName() const;
	
	bool IsReady() const;

	bool ReadStream( const ros::Time& time, StampedFeatures& features );

private:

	mutable Mutex _mutex;

	ros::NodeHandle _nodeHandle;
	LookupInterface _lookup;
	BroadcastInfoManager _infoManager;
	
	std::string _streamName;
	QueryMode _queryMode;

	ros::Subscriber _pushSub;
	ros::ServiceClient _pullClient;

	typedef std::map<ros::Time,StampedFeatures> StreamCache;
	StreamCache _featureCache;
	double _maxTimespan;

	bool PullStream( const ros::Time& time, StampedFeatures& f );
	bool ReadCached( const ros::Time& time, StampedFeatures& f ) const;

	void FeatureCallback( const broadcast::StampedFeatures::ConstPtr& msg );
	void CheckTimespan();
	double GetTimespan() const;
};

}