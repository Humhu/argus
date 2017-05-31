#pragma once

#include "odoscan/ScanMatcher.h"
#include "paraset/ParameterManager.hpp"

#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/random/MultivariateGaussian.hpp"
#include "argus_utils/synchronization/WorkerPool.h"
#include "argus_utils/synchronization/Semaphore.h"

namespace argus
{
// TODO Make class safer for multi-threaded usage
class MatchRestarter
{
public:

	MatchRestarter();

	void Initialize( ros::NodeHandle& ph,
	                 ScanMatcher::Ptr matcher );

	void SetDisplacementDistribution( const PoseSE3& disp,
	                                  const PoseSE3::CovarianceMatrix& cov );

	ScanMatchResult Match( const LaserCloudType::ConstPtr& key,
	                       const LaserCloudType::ConstPtr& tar,
	                       LaserCloudType::Ptr& aligned );

private:

	bool _twoDimensional;

	double _numStdDevs;
	MultivariateGaussian<> _guessDist;

	NumericParam _numRestarts;
	ScanMatcher::Ptr _matcher;
	WorkerPool _workers;

	PoseSE3 SampleInit( bool meanOnly );

	struct MatchJobInfo
	{
		LaserCloudType::ConstPtr key;
	    LaserCloudType::ConstPtr tar;
	    LaserCloudType::Ptr aligned;
		ScanMatchResult result;
		PoseSE3 guess;
	};

	void MatchJob( MatchJobInfo& info, Semaphore& sema );
};
}