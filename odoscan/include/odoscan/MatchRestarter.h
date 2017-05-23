#pragma once

#include "odoscan/ScanMatcher.h"
#include "argus_utils/geometry/PoseSE3.h"
#include "paraset/ParameterManager.hpp"
#include "argus_utils/random/UniformEllipse.hpp"

namespace argus
{

class MatchRestarter
{
  public:
	MatchRestarter();

	void Initialize(ros::NodeHandle &ph,
					ScanMatcher::Ptr matcher);

	ScanMatchResult Match(const LaserCloudType::ConstPtr &key,
						  const LaserCloudType::ConstPtr &tar,
						  const PoseSE3 &guess,
						  LaserCloudType::Ptr &aligned);

  private:
	bool _twoDimensional;
	UniformEllipse<> _guessDist;

	NumericParam _numRestarts;
	ScanMatcher::Ptr _matcher;

	PoseSE3 SampleInit();
};
}