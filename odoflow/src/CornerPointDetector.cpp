#include "odoflow/CornerPointDetector.h"
#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

CornerPointDetector::CornerPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph )
: InterestPointDetector( nh, ph )
{
	unsigned int initMaxFeats;
	GetParamRequired<unsigned int>( ph, "max_num_points", initMaxFeats );
	_featureMaxPoints.Initialize( ph, initMaxFeats, "max_num_points", "Maximum number of points to find" );
	_featureMaxPoints.AddCheck<GreaterThan>( 0 );
	_featureMaxPoints.AddCheck<IntegerValued>( ROUND_CEIL );

	double initMinQuality;
	GetParamRequired<double>( ph, "min_quality", initMinQuality );
	_featureMinQuality.Initialize( ph, initMinQuality, "min_quality", "Minimum feature quality threshold" );
	_featureMinQuality.AddCheck<GreaterThan>( 0 );

	double initMinSeparation;
	GetParamRequired<double>( ph, "min_separation", initMinSeparation );
	_featureMinSeparation.Initialize( ph, initMinSeparation, "min_separation", "Minimum feature separation (pix)" );
	_featureMinSeparation.AddCheck<GreaterThanOrEqual>( 0 );

	unsigned int initBlockDim;
	GetParamRequired<unsigned int>( ph, "block_dim", initBlockDim );
	_featureBlockDim.Initialize( ph, initBlockDim, "block_dim", "Feature covariation computation window size (pix)" );
	_featureBlockDim.AddCheck<GreaterThan>( 0 );
	_featureBlockDim.AddCheck<IntegerValued>( ROUND_CEIL );
	
	bool initUseHarris;
	GetParamRequired<bool>( ph, "use_harris", initUseHarris );
	_useHarris.Initialize( ph, initUseHarris, "use_harris", "Usage of Harris corner detector" );

	double initHarrisK;
	GetParamRequired<double>( ph, "harris_k", initHarrisK );
	_harrisK.Initialize( ph, initHarrisK, "harris_k", "Harris k value" );
	_harrisK.AddCheck<GreaterThanOrEqual>( 0 );
	
	bool initEnableRefine;
	GetParamRequired<bool>( ph, "enable_refinement", initEnableRefine );
	_enableRefine.Initialize( ph, initEnableRefine, "enable_refinement", "Enable subpixel feature refinement" );
	
	unsigned int initRefineDim;
	GetParamRequired<unsigned int>( ph, "refine_window_dim", initRefineDim );
	_refineWindowDim.Initialize( ph, initRefineDim, "refine_window_dim", "Refinement search window half-dim (pix)" );
	_refineWindowDim.AddCheck<GreaterThan>( 0 );
	_refineWindowDim.AddCheck<IntegerValued>( ROUND_CEIL );

	int initZeroDim;
	GetParamRequired<int>( ph, "refine_zero_dim", initZeroDim );
	_refineZeroDim.Initialize( ph, initZeroDim, "refine_zero_dim", "Refinement zero window half-dim (pix)" );
	_refineWindowDim.AddCheck<GreaterThanOrEqual>( -1 );
	_refineWindowDim.AddCheck<IntegerValued>( ROUND_CLOSEST );

	unsigned int initRefineIters;
	GetParamRequired<unsigned int>( ph, "refine_max_iters", initRefineIters );
	_refineMaxIters.Initialize( ph, initRefineIters, "refine_max_iters", "Maximum number of refinement iterations" );
	_refineMaxIters.AddCheck<GreaterThan>( 0 );
	_refineMaxIters.AddCheck<IntegerValued>( ROUND_CEIL );

	double initRefineEps;
	GetParamRequired<double>( ph, "refine_min_eps", initRefineEps );
	_refineMinEps.Initialize( ph, initRefineEps, "refine_min_eps", "Minimum refinement iteration improvement" );
	_refineMinEps.AddCheck<GreaterThanOrEqual>( 0 );

}

InterestPoints CornerPointDetector::FindInterestPoints( const cv::Mat& image )
{
	InterestPoints points;
	if( image.empty() ) { return points; }
	
	cv::goodFeaturesToTrack( image, 
	                         points, 
	                         _featureMaxPoints, 
	                         _featureMinQuality, 
	                         _featureMinSeparation, 
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
										 _refineMaxIters, _refineMinEps );
		cv::cornerSubPix( image, points, refineWindowSize, refineZeroSize, refineCriteria );
	}
	
	return points;
}

} // end namespace argus
