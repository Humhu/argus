#include "odoflow/FixedPointDetector.h"

#include "argus_utils/utils/ParamUtils.h"

#include <boost/foreach.hpp>

#include <iostream>

namespace argus
{

FixedPointDetector::FixedPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph )
: InterestPointDetector( nh, ph )
{
	unsigned int width = 3;
	unsigned int height = 3;
	std::vector<int> gridSize;
	if( privHandle.getParam( "detector/grid_size", gridSize ) )
	{
		width = static_cast<unsigned int>( gridSize[0] );
		height = static_cast<unsigned int>( gridSize[1] );
	}
	
	for( unsigned int i = 0; i < width; i++ )
	{
		double x = (i+1)/((double) width + 1);
		for( unsigned int j = 0; j < height; j++ )
		{
			double y = (j+1)/((double) height + 1);
			InterestPoint point( x, y );
			points.push_back( point );
		}
	}
	
}

InterestPoints FixedPointDetector::FindInterestPoints( const cv::Mat& image )
{
	
	cv::Size imgSize = image.size();
	InterestPoints scaledPoints;
	scaledPoints.reserve( points.size() );
	
	BOOST_FOREACH( const InterestPoint& point, points )
	{
		InterestPoint p( point.x*imgSize.width, point.y*imgSize.height );
		scaledPoints.push_back( p );
	}
		
	return scaledPoints;
}
	
}
