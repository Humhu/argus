#pragma once

#include "odoflow/MotionEstimator.h"
#include "paraset/ParameterManager.hpp"

namespace argus
{
/*! \brief Estimates a 2D rigid transformation. */
class RigidEstimator
{
public:

	typedef std::shared_ptr<RigidEstimator> Ptr;

	RigidEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph );

	bool EstimateMotion( InterestPoints& key,
	                     InterestPoints& tar,
						 std::vector<unsigned int>& inlierInds,
	                     PoseSE2& transform  );

private:

	NumericParam _logReprojThreshold;
	NumericParam _maxIters;
};
} // end namespace odoflow
