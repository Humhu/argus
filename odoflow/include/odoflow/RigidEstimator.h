#pragma once

#include "odoflow/MotionEstimator.h"

namespace odoflow
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
	                             argus::PoseSE3& transform );
	
private:
	
	double scale;
	double reprojThreshold;
	
};

} // end namespace odoflow
