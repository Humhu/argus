#include "odoflow/EightPointEstimator.h"

namespace odoflow
{
	
	EightPointEstimator::EightPointEstimator()
		: estimationMethod( EIGHT_POINT ), confidence( 0.99 ), outlierThreshold( 1.0 )
	{}
	
	bool EightPointEstimator::EstimateMotion( const InterestPoints& firstPoints,
											  const InterestPoints& secondPoints,
											  Transform& transform )
	{
		
		cv::Mat mask;
		
		cv::Mat fundamental = 
			cv::findFundamentalMat( firstPoints, secondPoints, ConvertMethod( estimationMethod )
									outlierThreshold, confidence, mask );
		
		
			
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
