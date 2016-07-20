#include "odoflow/FixedPointDetector.h"
#include "argus_utils/utils/ParamUtils.h"

#include <boost/foreach.hpp>
#include <iostream>
#include <sstream>

namespace argus
{

FixedPointDetector::FixedPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph )
: InterestPointDetector( nh, ph ), 
_cachedGridSize( 0, 0 ),
_gridSize( ph, "grid_size" )
{

	std::vector<unsigned int> heights, widths;
	GetParamRequired<std::vector<unsigned int>>( privHandle, "grid_heights", heights );
	GetParamRequired<std::vector<unsigned int>>( privHandle, "grid_widths", widths );
	if( heights.size() != widths.size() )
	{
		throw std::runtime_error( "Grid heights and widths must be same length." );
	}

	for( unsigned int i = 0; i < heights.size(); i++ )
	{
		std::stringstream ss;
		ss << "[" << widths[i] << ", " << heights[i] << "]";
		GridSize gs( widths[i], heights[i] );
		_gridSize.AddSetting( gs, ss.str() );
	}
	
	UpdateInterestPoints();
}

InterestPoints FixedPointDetector::FindInterestPoints( const cv::Mat& image )
{
	UpdateInterestPoints();

	cv::Size imgSize = image.size();
	InterestPoints scaledPoints;
	scaledPoints.reserve( _cachedGrid.size() );
	
	BOOST_FOREACH( const InterestPoint& point, _cachedGrid )
	{
		InterestPoint p( point.x*imgSize.width, point.y*imgSize.height );
		scaledPoints.push_back( p );
	}
		
	return scaledPoints;
}

void FixedPointDetector::UpdateInterestPoints()
{
	std::pair<unsigned int, unsigned int> gridSize = _gridSize.GetValue();
	if( gridSize == _cachedGridSize ) { return; }

	_cachedGrid.clear();
	unsigned int width = gridSize.first;
	unsigned int height = gridSize.second;
	_cachedGrid.reserve( width * height );
	for( unsigned int i = 0; i < width; i++ )
	{
		double x = (i+1)/((double) width + 1);
		for( unsigned int j = 0; j < height; j++ )
		{
			double y = (j+1)/((double) height + 1);
			_cachedGrid.emplace_back( x, y );
		}
	}

	_cachedGridSize = gridSize;
}

}
