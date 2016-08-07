#pragma once

#include "broadcast/BroadcastMultiReceiver.h"
#include "covreg/CovarianceEstimator.h"
#include "covreg/QueryCovariance.h"

#include "argus_utils/synchronization/SynchronizationTypes.h"

#include <deque>

namespace argus
{

class CovarianceManager
{
public:

	CovarianceManager();

	void Initialize( const std::string& sourceName,
	                 ros::NodeHandle& ph );

	void SetUpdateTopic( const std::string& topic );

	bool IsReady() const;

	unsigned int OutputDim() const;

	MatrixType EstimateCovariance( const ros::Time& time );

private:

	ros::NodeHandle _nodeHandle;

	ros::ServiceServer _queryServer;

	std::string _sourceName;
	ros::Subscriber _paramSub;
	std::shared_ptr<CovarianceEstimator> _estimator;
	BroadcastMultiReceiver _receiver;

	Mutex _estimatorMutex;

	void ParamCallback( const covreg::CovarianceEstimatorInfo::ConstPtr& msg );

	bool QueryCallback( covreg::QueryCovariance::Request& request,
	                    covreg::QueryCovariance::Response& response );

};

}