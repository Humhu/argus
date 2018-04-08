#pragma once

#include "odoflow/OdoflowCommon.h"
#include "paraset/ParameterManager.hpp"

namespace argus
{
/*! \brief Estimates a 2D rigid transformation with RANSAC outlier
 * rejection.
 */
class RigidEstimator
{
public:

	typedef std::shared_ptr<RigidEstimator> Ptr;

	RigidEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph );

	/*! \brief Finds a transformation that takes tar points to key
	 * points. Returns success and a list of inlier point indices.
	 */
	bool EstimateMotion( InterestPoints& key,
	                     InterestPoints& tar,
	                     std::vector<unsigned int>& inlierInds,
	                     PoseSE2& transform );

private:

	NumericParam _logReprojThreshold;
	NumericParam _maxIters;
};
}
