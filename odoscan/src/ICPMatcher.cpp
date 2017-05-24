#include "odoscan/ICPMatcher.h"

namespace argus
{

ICPMatcher::ICPMatcher() {}

void ICPMatcher::InitializeDerived(ros::NodeHandle &ph)
{
	_useReciprocalCorrespond.InitializeAndRead(ph, false, "use_reciprocal_correspond",
											   "Use reciprocal correspondences.");
}

ScanMatcher::MatcherType::Ptr ICPMatcher::CreateMatcher()
{
	ExposedICP::Ptr icp = std::make_shared<ExposedICP>();
	icp->setUseReciprocalCorrespondences(_useReciprocalCorrespond);
	return icp;
}
}