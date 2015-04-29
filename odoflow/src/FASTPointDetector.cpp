#include "odoflow/FASTPointDetector.h"

#include <opencv2/features2d.hpp>

#include <boost/foreach.hpp>

#include <iostream>

namespace odoflow
{

	FASTPointDetector::FASTPointDetector()
		: intensityThreshold( 20 ), useNonMaxSuppression( true ), 
		detectorType( FAST_9_16 ), maxPoints( 40 )
	{}
	
	InterestPoints FASTPointDetector::FindInterestPoints( const cv::Mat& image )
	{
		std::vector<cv::KeyPoint> keypoints;
		cv::FAST( image, keypoints, intensityThreshold, useNonMaxSuppression, 
				  ConvertDetectorType(detectorType) );
		
		unsigned int numPicks = std::min( maxPoints, keypoints.size() );
		
		std::vector<unsigned int> indices( keypoints.size() );
		std::iota( indices.begin(), indices.end(), 0 );
		std::random_shuffle( indices.begin(), indices.end() );
		
		InterestPoints points( numPicks );
		for( unsigned int i = 0; i < numPicks; i++ )
		{
			points[i] = keypoints[indices[i]].pt;
		}
		
		return points;
	}
	
	void FASTPointDetector::SetMaxPoints( int m )
	{
		maxPoints = (unsigned int) m;
	}
	
	void FASTPointDetector::SetIntensityThreshold( int thresh )
	{
		intensityThreshold = thresh;
	}
	
	void FASTPointDetector::SetNonMaxSuppression( bool enable )
	{
		useNonMaxSuppression = enable;
	}
	
	void FASTPointDetector::SetDetectorType( DetectorType type )
	{
		detectorType = type;
	}
	
	int FASTPointDetector::ConvertDetectorType( DetectorType type )
	{
		switch( type )
		{
			case FAST_9_16:
				return cv::FastFeatureDetector::TYPE_9_16;
			case FAST_7_12:
				return cv::FastFeatureDetector::TYPE_7_12;
			case FAST_5_8:
				return cv::FastFeatureDetector::TYPE_5_8;
			default:
				throw std::runtime_error( "Invalid FAST detector type." );
		}
	}
	
}
