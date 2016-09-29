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
	unsigned int intensityThreshold;
	GetParamRequired( ph, "intensity_threshold", intensityThreshold );
	_intensityThreshold.Initialize( ph, intensityThreshold, "intensity_threshold",
	                                "Minimum central pixel intensity difference" );
	_intensityThreshold.AddCheck<GreaterThanOrEqual>( 0 );
	_intensityThreshold.AddCheck<LessThanOrEqual>( 255 );
	_intensityThreshold.AddCheck<IntegerValued>( ROUND_CLOSEST );

	bool useNMS;
	GetParamRequired<bool>( ph, "enable_nonmax_suppression", useNMS );
	_enableNMS.Initialize( ph, useNMS, "enable_nonmax_suppression", 
	                       "Usage of non-maximum suppression" );

	unsigned int maxNumPoints;
	GetParamRequired( ph, "max_num_points", maxNumPoints );
	_maxNumPoints.Initialize( ph, maxNumPoints, "max_num_points",
	                          "Maximum number of points to find" );
	_maxNumPoints.AddCheck<GreaterThanOrEqual>( 0 );
	_maxNumPoints.AddCheck<IntegerValued>( ROUND_CEIL );

	std::string initType;
	GetParamRequired<std::string>( ph, "detector_type", initType );
	_detectorType.Initialize( ph, initType, "detector_type",
	                          "FAST detector type" );
	// TODO Set type at runtime?
}

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
