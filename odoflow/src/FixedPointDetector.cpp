#include "odoflow/FixedPointDetector.h"
#include "argus_utils/utils/ParamUtils.h"

#include <boost/foreach.hpp>
#include <iostream>
#include <sstream>

namespace argus
{

FixedPointDetector::FixedPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph )
: InterestPointDetector( nh, ph ), 
_cachedGridSize( 0, 0 )
{

	unsigned int gridDim;
	GetParamRequired<unsigned int>( privHandle, "grid_dim", gridDim );
	_gridDim.Initialize( ph, gridDim, "grid_dim", "Interest point grid width and height." );
	_gridDim.AddCheck<GreaterThan>( 0 );

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
	GridSize gridSize( _gridDim, _gridDim );

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
