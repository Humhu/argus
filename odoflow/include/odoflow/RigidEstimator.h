#pragma once

#include "odoflow/MotionEstimator.h"
#include "paraset/ParameterManager.hpp"

namespace argus
{

/*! \brief Estimates a 2D rigid transformation. */
class RigidEstimator
	: public MotionEstimator
{
public:
	
	typedef std::shared_ptr<RigidEstimator> Ptr;
	
	RigidEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	virtual bool EstimateMotion( const InterestPoints& srcPoints,
	                             const InterestPoints& dstPoints,
	                             std::vector<uchar>& inliers,
	                             PoseSE3& transform,
	                             PoseSE2& frameTransform );
	
private:
	
	double _scale;
	FloatParameter _reprojThreshold;
	IntegerParameter _maxIters;
	
};

} // end namespace odoflow
