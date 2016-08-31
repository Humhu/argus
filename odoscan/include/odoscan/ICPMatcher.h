#pragma once

#include "odoscan/ScanMatcher.h"

#include <pcl/registration/icp.h>

namespace argus
{

class ICPMatcher
: public ScanMatcher
{
public:

	typedef std::shared_ptr<ICPMatcher> Ptr;

	ICPMatcher();

private:

	typedef pcl::IterativeClosestPoint<LaserPointType, LaserPointType> ICPType;

	BooleanParam _useReciprocalCorrespond;

	virtual void InitializeDerived( ros::NodeHandle& ph );
	virtual MatcherType::Ptr CreateMatcher();
};

}