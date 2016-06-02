#pragma once

#include <Eigen/Geometry>

#include "odoflow/InterestPointDetector.h"
#include "argus_utils/geometry/PoseSE3.h"

namespace argus
{
	
typedef Eigen::Transform<double, 3, Eigen::Isometry> Transform;

class MotionEstimator
{
public:
	
	typedef std::shared_ptr<MotionEstimator> Ptr;
			
	MotionEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: nodeHandle( nh ), privHandle( ph ) {}
	
	/*! \brief Estimates the transform that turns srcPoints into dstPoints.
	 * Returns success. */
	virtual bool EstimateMotion( const InterestPoints& srcPoints,
	                             const InterestPoints& dstPoints,
	                             std::vector<uchar>& inliers,
	                             argus::PoseSE3& transform ) = 0;

	/*! \brief Rectifies points into normalized camera coordinates. */
	// TODO Use undistortion parameters?
	InterestPoints RectifyPoints( const InterestPoints& points );
	
protected:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
};

} // end namespace odoflow
