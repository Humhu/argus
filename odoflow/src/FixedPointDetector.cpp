#include "odoflow/FixedPointDetector.h"

#include <boost/foreach.hpp>

#include <iostream>

namespace odoflow
{

	FixedPointDetector::FixedPointDetector() {}
	
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
	
	void FixedPointDetector::SetPoints( const InterestPoints& pts )
	{
		points = pts;
	}
	
}
