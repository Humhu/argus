#pragma once

#include "odoscan/OdoscanCommon.h"
#include "paraset/ParameterManager.hpp"
#include <pcl/sample_consensus/ransac.h>

namespace argus
{

class DegeneracyDetector
{
public:

	DegeneracyDetector();

	void Initialize( ros::NodeHandle& nh );

	bool HasDegeneracy( const LaserCloudType::ConstPtr& cloud );

private:

	typedef pcl::RandomSampleConsensus<LaserPointType> RANSAC;

	BooleanParam _checkPlane;
	BooleanParam _checkSphere;
        BooleanParam _checkLine;
        BooleanParam _checkCircle;
	NumericParam _inlierThreshold;
	NumericParam _maxIterations;
	NumericParam _maxDegeneracyRatio;

	unsigned int CountInliers( RANSAC& ransac );
};

}
