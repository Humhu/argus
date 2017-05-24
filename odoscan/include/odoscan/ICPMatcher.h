#pragma once

#include "odoscan/ScanMatcher.h"

#include <pcl/registration/icp.h>

namespace argus
{

class ExposedICP
: public ExposedMatcher, public pcl::IterativeClosestPoint<LaserPointType, LaserPointType, float>
{
public:
	typedef std::shared_ptr<ExposedICP> Ptr;

	unsigned int CountCorrespondences( double dist )
	{
		unsigned int count = 0;
		for (uint32_t i = 0; i < this->correspondences_->size(); i++)
		{
			pcl::Correspondence currCorr = (*this->correspondences_)[i];
			if( currCorr.index_match != -1 && currCorr.distance < dist ) { ++count; }
		}
		return count;
	}

	Registrar& ToRegistrar() { return *this; }
};

class ICPMatcher
	: public ScanMatcher
{
  public:
	typedef std::shared_ptr<ICPMatcher> Ptr;

	ICPMatcher();

  private:

	BooleanParam _useReciprocalCorrespond;

	virtual void InitializeDerived(ros::NodeHandle &ph);
	virtual MatcherType::Ptr CreateMatcher();
};
}