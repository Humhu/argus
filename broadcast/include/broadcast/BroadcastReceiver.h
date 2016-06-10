#pragma once

#include <map>
#include <ros/ros.h>
#include <argus_utils/utils/LinalgTypes.h>
#include <argus_utils/synchronization/SynchronizationTypes.h>

#include "broadcast/StampedFeatures.h"
#include "broadcast/BroadcastInfoManager.h"

namespace argus
{

/*! \brief Wraps looking up, receiving, and caching a feature broadcast. */
class BroadcastReceiver
{
public:


	BroadcastReceiver( const std::string& streamName,
	                   unsigned int cacheSize,
	                   unsigned int incomingQueueSize = 10,
	                   const std::string& topic = "features_raw" );

	bool HasReceived() const;
	VectorType GetClosestReceived( const ros::Time& time ) const;

private:

	typedef std::map<ros::Time,VectorType> MapType;

	void FeatureCallback( const broadcast::StampedFeatures::ConstPtr& msg );
	
	mutable Mutex _mutex;

	ros::NodeHandle _nodeHandle;
	LookupInterface _lookup;
	BroadcastInfoManager _infoManager;
	std::string _streamName;
	MapType _featureMap;
	unsigned int _maxMapSize;
	ros::Subscriber _featSub;
};

}