#ifndef _OFLOW_MOTION_ESTIMATOR_H_
#define _OFLOW_MOTION_ESTIMATOR_H_

#include <Eigen/Geometry>

#include "odoflow/InterestPointDetector.h"
#include "argus_utils/PoseSE3.h"

namespace odoflow
{
	
	typedef Eigen::Transform<double, 3, Eigen::Isometry> Transform;
	
	class MotionEstimator
	{
	public:
		
		typedef std::shared_ptr<MotionEstimator> Ptr;
				
		MotionEstimator();
		
		/*! \brief Estimates the camera transform between the two sets of
		 * corresponding image points. Returns success. */
		virtual bool EstimateMotion( const InterestPoints& firstPoints,
									 const InterestPoints& secondPoints,
									 argus_utils::PoseSE3& transform ) = 0;

		/*! \brief Rectifies points into normalized camera coordinates. */
		// TODO Use undistortion parameters
		InterestPoints RectifyPoints( const InterestPoints& points );
		
	protected:
		
		cv::Mat cameraMatrix;
									 
	};
}

#endif
