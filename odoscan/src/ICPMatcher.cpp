#include "odoscan/ICPMatcher.h"

namespace argus
{

ICPMatcher::ICPMatcher() {}

void ICPMatcher::InitializeDerived( ros::NodeHandle& ph )
{
	bool useRecipCorr;
	GetParamRequired( ph, "use_reciprocal_correspond", useRecipCorr );
	_useReciprocalCorrespond.Initialize( ph, useRecipCorr, "use_reciprocal_correspond",
	                                     "Use reciprocal correspondences." );
}

ScanMatcher::MatcherType::Ptr ICPMatcher::CreateMatcher()
{
	ICPType::Ptr icp = boost::make_shared<ICPType>();
	icp->setUseReciprocalCorrespondences( _useReciprocalCorrespond );
	return icp;
}

}