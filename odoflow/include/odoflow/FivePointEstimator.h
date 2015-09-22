#pragma once

#include "odoflow/MotionEstimator.h"

// #include <opencv2/calib3d/calib3d.hpp>

namespace odoflow
{

// TODO This is horribly broken and needs to be fixed
/*! \brief Estimates camera motion between frames using the 5-point algorithm
 * by Nister. Parameters in private namespace are:
 * estimator:
 *   residual_threshold: [double] {0-inf} (0.002)
 */
class FivePointEstimator
	: public MotionEstimator
{
public:
	
	typedef std::shared_ptr<FivePointEstimator> Ptr;
	
	FivePointEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	virtual bool EstimateMotion( const InterestPoints& firstPoints,
									const InterestPoints& secondPoints,
									argus_utils::PoseSE3& transform );
	
	/*! \brief Set the method used to estimate the essential matrix. */
	void SetMethod( Method method );
	
	/*! \brief Set the RANSAC max epipolar distance in pixels and desired confidence. */
	void SetFittingParameters( double prob, double thresh=1.0 );
	
	/*! \brief Set the pixel to output scale factor. */
	void SetOutputScale( double scale );
	
private:
	
	
	Method estimationMethod;
	double confidence;
	double outlierThreshold;
	
	double residualThreshold;

	// Converts pixel distances to output scale
	double outputScale;

	int ConvertMethod( Method method );
	
};
	
} // end namespace odoflow
