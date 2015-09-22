#include "odoflow/CornerPointDetector.h"

namespace odoflow
{

CornerPointDetector::CornerPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph )
: InterestPointDetector( nh, ph )
{
	privHandle.param( "detector/max_num_points", featureMaxPoints, 500 );
	privHandle.param( "detector/min_quality", featureQuality, 0.01 );
	privHandle.param( "detector/min_separation", featureSeparation, 10.0 );
	privHandle.param( "detector/block_size", featureBlockSize, 3 );
	
	int refineWindowDim;
	privHandle.param( "detector/window_size", refineWindowDim, 10 );
	refineWindowSize = cv::Size( refineWindowDim, refineWindowDim );
	
	privHandle.param( "detector/use_harris", featureUseHarris, false );
	if( featureUseHarris )
	{
		privHandle.param( "detector/harris_k", featureHarrisK, 0.04 );
	}
	
	privHandle.param( "detector/refine_enable", refineEnable, true );
	
	if( refineEnable )
	{
		int refineMaxIters;
		double refineMinEps;
		privHandle.param( "detector/refine_max_iters", refineMaxIters, 20 );
		privHandle.param( "detector/refine_min_eps", refineMinEps, 0.03 );
		refineTermCriteria = cv::TermCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 
											   refineMaxIters, refineMinEps );
	}

}

InterestPoints CornerPointDetector::FindInterestPoints( const cv::Mat& image )
{
	InterestPoints points;
	cv::goodFeaturesToTrack( image, points, featureMaxPoints, featureQuality, featureSeparation, 
								cv::noArray(), featureBlockSize, featureUseHarris, featureHarrisK );
	
	// TODO Subpixel refinement
	
	return points;
}

} // end namespace odoflow
