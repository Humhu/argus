#include "odoflow/FASTPointDetector.h"

#include <boost/foreach.hpp>

#include <iostream>

namespace odoflow
{

	FASTPointDetector::FASTPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: InterestPointDetector( nh, ph )
	{
		int intensityThreshold;
		bool useNMS;
		privHandle.param( "detector/intensity_threshold", intensityThreshold, 20 );
		privHandle.param( "detector/enable_non_max_suppression", useNMS, true );
		
		std::string type;
		privHandle.param<std::string>( "detector/detector_type", type, "FAST_9_16" );
		int t;
		if( type == "FAST_5_8" )
		{
			t = cv::FastFeatureDetector::TYPE_5_8;
		}
		else if( type == "FAST_7_12" )
		{
			t = cv::FastFeatureDetector::TYPE_7_12;
		}
		else if( type == "FAST_9_16" )
		{
			t = cv::FastFeatureDetector::TYPE_9_16;
		}
		else
		{
			ROS_ERROR_STREAM( "Invalid FAST detector specified. Must be FAST_5_8, FAST_7_12, or FAST_9_16" );
			exit( -1 );
		}
		
		detector = cv::FastFeatureDetector::create( intensityThreshold, useNMS, t );
		
	}
	
	bool CompareKeypoints( const cv::KeyPoint& a, const cv::KeyPoint& b )
	{
		return a.response < b.response;
	}
	
	InterestPoints FASTPointDetector::FindInterestPoints( const cv::Mat& image )
	{
		std::vector<cv::KeyPoint> keypoints;
		detector->detect( image, keypoints );
		
		std::sort( keypoints.begin(), keypoints.end(), CompareKeypoints );
		
		InterestPoints points;
		points.reserve( keypoints.size() );
		for( unsigned int i = 0; i < keypoints.size(); i++ )
		{
			points.push_back( keypoints[i].pt );
		}
		return points;
	}
	
}
