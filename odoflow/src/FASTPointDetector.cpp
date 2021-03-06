#include "odoflow/FASTPointDetector.h"
#include "argus_utils/utils/ParamUtils.h"

#include <boost/foreach.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

namespace argus
{
FASTPointDetector::FASTPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: InterestPointDetector( nh, ph )
{
	_intensityThreshold.InitializeAndRead( ph, 50, "intensity_threshold",
	                                       "Minimum central pixel intensity difference" );
	_intensityThreshold.AddCheck<GreaterThanOrEqual>( 0 );
	_intensityThreshold.AddCheck<LessThanOrEqual>( 255 );
	_intensityThreshold.AddCheck<IntegerValued>( ROUND_CLOSEST );

	_enableNMS.InitializeAndRead( ph, true, "enable_nonmax_suppression",
	                              "Usage of non-maximum suppression" );

	_maxNumPoints.InitializeAndRead( ph, 255, "max_num_points",
	                                 "Maximum number of points to find" );
	_maxNumPoints.AddCheck<GreaterThanOrEqual>( 0 );
	_maxNumPoints.AddCheck<IntegerValued>( ROUND_CEIL );

	_detectorType.InitializeAndRead( ph, "FAST_7_12", "detector_type",
	                                 "FAST detector type" );
}

/*! \brief Comparator used for sorting keypoints by responses
 */
bool CompareKeypoints( const cv::KeyPoint& a, const cv::KeyPoint& b )
{
	return a.response < b.response;
}

InterestPoints FASTPointDetector::FindInterestPoints( const cv::Mat& image )
{
	std::vector<cv::KeyPoint> keypoints;
	cv::FAST( image,
	          keypoints,
	          _intensityThreshold,
	          _enableNMS,
	          StringToDetector( _detectorType ) );

	std::sort( keypoints.begin(), keypoints.end(), CompareKeypoints );

	InterestPoints points;
	unsigned int toCopy = std::min( keypoints.size(), (size_t) _maxNumPoints.GetValue() );
	points.reserve( toCopy );
	for( unsigned int i = 0; i < toCopy; i++ )
	{
		points.push_back( keypoints[i].pt );
	}
	return points;
}

int FASTPointDetector::StringToDetector( const std::string& str )
{
	if( str == "FAST_5_8" )
	{
		return cv::FastFeatureDetector::TYPE_5_8;
	}
	else if( str == "FAST_7_12" )
	{
		return cv::FastFeatureDetector::TYPE_7_12;
	}
	else if( str == "FAST_9_16" )
	{
		return cv::FastFeatureDetector::TYPE_9_16;
	}
	else
	{
		throw std::runtime_error( "Invalid FAST detector " + str +
		                          " specified. Must be FAST_5_8, FAST_7_12, or FAST_9_16" );
	}
}
}
