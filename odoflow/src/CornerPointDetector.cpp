#include "odoflow/CornerPointDetector.h"
#include "argus_utils/utils/ParamUtils.h"

namespace argus
{
CornerPointDetector::CornerPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: InterestPointDetector( nh, ph )
{
	_featureMaxPoints.InitializeAndRead( ph, 100, "max_num_points", "Maximum number of points to find" );
	_featureMaxPoints.AddCheck<GreaterThan>( 0 );
	_featureMaxPoints.AddCheck<IntegerValued>( ROUND_CEIL );

	_featureMinQuality.InitializeAndRead( ph, 0.05, "min_quality", "Minimum feature quality threshold" );
	_featureMinQuality.AddCheck<GreaterThan>( 0 );

	_featureMinSeparation.InitializeAndRead( ph, 10.0, "min_separation", "Minimum feature separation (normalized)" );
	_featureMinSeparation.AddCheck<GreaterThanOrEqual>( 0 );
	_featureMinSeparation.AddCheck<LessThanOrEqual>( 1.0 );

	_featureBlockDim.InitializeAndRead( ph, 3, "block_dim", "Feature covariation computation window size (pix)" );
	_featureBlockDim.AddCheck<GreaterThan>( 0 );
	_featureBlockDim.AddCheck<IntegerValued>( ROUND_CEIL );

	_useHarris.InitializeAndRead( ph, false, "use_harris", "Usage of Harris corner detector" );

	_harrisK.InitializeAndRead( ph, 0.04, "harris_k", "Harris k value" );
	_harrisK.AddCheck<GreaterThanOrEqual>( 0 );

	_enableRefine.InitializeAndRead( ph, true, "enable_refinement", "Enable subpixel feature refinement" );

	_refineWindowDim.InitializeAndRead( ph, 3, "refine_window_dim", "Refinement search window half-dim (pix)" );
	_refineWindowDim.AddCheck<GreaterThan>( 0 );
	_refineWindowDim.AddCheck<IntegerValued>( ROUND_CEIL );

	_refineZeroDim.InitializeAndRead( ph, -1, "refine_zero_dim", "Refinement zero window half-dim (pix)" );
	_refineWindowDim.AddCheck<GreaterThanOrEqual>( -1 );
	_refineWindowDim.AddCheck<IntegerValued>( ROUND_CLOSEST );

	_refineMaxIters.InitializeAndRead( ph, 5, "refine_max_iters", "Maximum number of refinement iterations" );
	_refineMaxIters.AddCheck<GreaterThan>( 0 );
	_refineMaxIters.AddCheck<IntegerValued>( ROUND_CEIL );

	_refineMinLogEps.InitializeAndRead( ph, -4, "refine_min_log_eps", "Minimum refinement iteration log10 improvement" );
}

InterestPoints CornerPointDetector::FindInterestPoints( const cv::Mat& image )
{
	InterestPoints points;
	if( image.empty() ) { return points; }

	double imageScale = ( image.size().width + image.size().height ) * 0.5;
	double separation = _featureMinSeparation * imageScale;
	cv::goodFeaturesToTrack( image,
	                         points,
	                         _featureMaxPoints,
	                         _featureMinQuality,
	                         separation,
	                         cv::noArray(),
	                         _featureBlockDim,
	                         _useHarris,
	                         _harrisK );

	if( points.size() == 0 ) { return points; }

	// TODO Subpixel refinement
	if( _enableRefine )
	{
		cv::Size refineWindowSize( _refineWindowDim, _refineWindowDim );
		cv::Size refineZeroSize( _refineZeroDim, _refineZeroDim );
		cv::TermCriteria refineCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
		                                 _refineMaxIters,
		                                 std::pow( 10, _refineMinLogEps ) );
		cv::cornerSubPix( image, points, refineWindowSize, refineZeroSize, refineCriteria );
	}

	return points;
}
} // end namespace argus
