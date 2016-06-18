#pragma once

#include "broadcast/BroadcastReceiver.h"
#include "covreg/CovarianceEstimator.h"

#include "argus_utils/synchronization/SynchronizationTypes.h"

#include <deque>

namespace argus
{

class CovarianceManager
{
public:

	CovarianceManager();

	void Initialize( const std::string& sourceName,
	                 ros::NodeHandle& ph,
	                 const std::string& subName );

	void Initialize( const std::string& sourceName,
	                 const YAML::Node& info,
	                 double cacheTime );

	void SetUpdateTopic( const std::string& topic );

	bool IsReady() const;

	unsigned int OutputDim() const;

	MatrixType EstimateCovariance( const ros::Time& time );

private:

	ros::NodeHandle _nodeHandle;

	std::string _sourceName;
	ros::Subscriber _paramSub;
	std::shared_ptr<CovarianceEstimator> _estimator;
	std::deque<BroadcastReceiver> _receivers;

	Mutex _estimatorMutex;

	void ParamCallback( const covreg::CovarianceEstimatorInfo::ConstPtr& msg );

};

}