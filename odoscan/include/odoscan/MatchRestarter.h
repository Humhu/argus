#pragma once

#include "odoscan/ScanMatcher.h"
#include "argus_utils/geometry/PoseSE3.h"
#include "paraset/ParameterManager.hpp"
#include "argus_utils/random/MultivariateGaussian.hpp"

namespace argus
{
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

	PoseSE3 SampleInit( bool meanOnly );
};
}