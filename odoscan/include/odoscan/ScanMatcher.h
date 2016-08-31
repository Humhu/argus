#pragma once

#include "argus_utils/geometry/PoseSE3.h"
#include "odoscan/OdoscanCommon.h"

#include "paraset/ParameterManager.hpp"

#include <pcl/registration/registration.h>
#include <memory>

namespace argus
{

/*! \brief Base for all classes that find the transform between two point cloud. Provides
 * common set of parameters. */
class ScanMatcher
{
public:

	typedef std::shared_ptr<ScanMatcher> Ptr;
	typedef pcl::Registration<LaserPointType, LaserPointType> MatcherType;

	ScanMatcher();
	virtual ~ScanMatcher();

	void Initialize( ros::NodeHandle& ph );

	/*! \brief Find the transform that turns tar into key. Gives the transformed version
	 * of tar. Returns success. */
	bool Match( const LaserCloudType::ConstPtr& key, 
	            const LaserCloudType::ConstPtr& tar, 
	            PoseSE3& pose,
	            LaserCloudType::Ptr& aligned );

protected:

	virtual void InitializeDerived( ros::NodeHandle& ph ) = 0;

	/*! \brief Creates and returns a scan matching object. Should be implemented by derived class. 
	 Should set any derived class runtime parameters. */
	virtual MatcherType::Ptr CreateMatcher() = 0;

private:

	NumericParam _maxIters;
	NumericParam _ransacIters;
	NumericParam _ransacThreshold;
	NumericParam _maxCorrespondDist;
	NumericParam _minTransformEps;
	NumericParam _minObjectiveEps;
	NumericParam _maxError;

};

}