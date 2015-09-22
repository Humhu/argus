#include "odoflow/EightPointEstimator.h"

namespace odoflow
{
	
	EightPointEstimator::EightPointEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: MotionEstimator( nh, ph ), estimationMethod( EIGHT_POINT )
	{
		ph.param( "motion_estimator/confidence", confidence, 0.99 );
		ph.param( "motion_estimator/outlier_threshold", outlierThreshold, 1.0 );
		std::string methodName;
		ph.param( "motion_estimator/method_name", methodName, "8_point" );
		
		// TODO
		if( methodName == "7_point" )
		{
			
		}
		else if( methodName == "8_point" )
		{
			
		}
		else if( methodName == "fm_ransac" )
		{
			
		}
		else if( methodName == "fm_lmeds" )
		{
		
		}
		else
		{
			ROS_ERROR_STREAM( "Invalid 8-point method name given. Must be 7_point " +
							  "8_point, fm_ransac, or fm_lmeds" );
		}
		
	}
	
	bool EightPointEstimator::EstimateMotion( const InterestPoints& firstPoints,
											  const InterestPoints& secondPoints,
											  Transform& transform )
	{
		
		cv::Mat mask;
		
		cv::Mat fundamental = 
			cv::findFundamentalMat( firstPoints, secondPoints, ConvertMethod( estimationMethod )
									outlierThreshold, confidence, mask );
		// TODO
		
			
	}
	
	int EightPointEstimator::ConvertMethod( Method method )
	{
		switch( method )
		{
			case SEVEN_POINT:
				return cv::FM_7POINT;
			case EIGHT_POINT:
				return cv::FM_8POINT;
			case EIGHT_RANSAC:
				return cv::FM_RANSAC;
			case EIGHT_MEDS:
				return cv::FM_LMEDS;
			default:
				throw std::runtime_error( "Invalid Eight Point method." );
		}
	}
			
	
}
