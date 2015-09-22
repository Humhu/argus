#pragma once

#include "odoflow/MotionEstimator.h"

namespace odoflow
{

/*! \brief Motion estimator using the OpenCV 8-point algorithm. 
 * Params are under the private namespace:
 * motion_estimator:
 *   confidence: [double] (RANSAC confidence)
 *   outlier_thresold: [double] (RANSAC outlier threshold)
 *   method_name: [7_point, 8_point, fm_ransac, fm_lmeds]
 */
class EightPointEstimator
	: public MotionEstimator
{
public:
	
	typedef std::shared_ptr<EightPointEstimator> Ptr;
	
	EightPointEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	virtual bool EstimateMotion( const InterestPoints& firstPoints,
									const InterestPoints& secondPoints,
									Transform& transform );
	
private:
	
	int estimationMethod;
	double confidence;
	double outlierThreshold;
	
	int ConvertMethod( Method method );
	
};

} // end namespace odoflow

