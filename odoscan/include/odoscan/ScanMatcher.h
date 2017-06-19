#pragma once

#include "argus_utils/geometry/PoseSE3.h"
#include "odoscan/OdoscanCommon.h"

#include "paraset/ParameterManager.hpp"

#include <pcl/registration/registration.h>
#include <memory>

namespace argus
{

class ExposedMatcher
{
  public:
	typedef std::shared_ptr<ExposedMatcher> Ptr;
	typedef pcl::Registration<LaserPointType, LaserPointType, float> Registrar;

	ExposedMatcher() {}
	virtual ~ExposedMatcher() {}

	virtual LaserCloudType::Ptr GetInliers(double dist) = 0;
	virtual Registrar &ToRegistrar() = 0;
};

struct ScanMatchResult
{
	bool success;
	double fitness;
	PoseSE3 transform;
	LaserCloudType::Ptr inliers;
};

/*! \brief Base for all classes that find the transform between two point cloud. Provides
 * common set of parameters. */
class ScanMatcher
{
  public:
	typedef std::shared_ptr<ScanMatcher> Ptr;
	typedef ExposedMatcher MatcherType;

	ScanMatcher();
	virtual ~ScanMatcher();

	void Initialize(ros::NodeHandle &ph);

	/*! \brief Find the transform that turns tar into key. Gives the transformed version
	 * of tar. Returns success. */
	ScanMatchResult Match(const LaserCloudType::ConstPtr &key,
						  const LaserCloudType::ConstPtr &tar,
						  const PoseSE3 &pose,
						  LaserCloudType::Ptr &aligned);

  private:
	NumericParam _maxIters;
	NumericParam _ransacIters;
	NumericParam _ransacThreshold;
	NumericParam _maxCorrespondDist;
	NumericParam _logMinTransformEps;
	NumericParam _logMinObjectiveEps;
	NumericParam _minNumPoints;
	
	virtual void InitializeDerived(ros::NodeHandle &ph) = 0;

	/*! \brief Creates and returns a scan matching object. Should be implemented by derived class. 
	 Should set any derived class runtime parameters. */
	virtual MatcherType::Ptr CreateMatcher() = 0;
};
}