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
	
	virtual ~MotionEstimator() {}

	/*! \brief Estimates the transform that turns tar into key.
	 * Returns success. */
	virtual bool EstimateMotion( FrameInterestPoints& key,
	                             FrameInterestPoints& tar,
	                             argus::PoseSE3& transform ) = 0;

	/*! \brief Rectifies points into normalized camera coordinates. */
	// TODO Use undistortion parameters?
	InterestPoints RectifyPoints( const InterestPoints& points );
	
protected:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
};

} // end namespace odoflow
