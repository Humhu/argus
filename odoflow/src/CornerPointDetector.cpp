#include "odoflow/CornerPointDetector.h"

namespace argus
{

CornerPointDetector::CornerPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph )
: InterestPointDetector( nh, ph )
{
	privHandle.param( "max_num_points", featureMaxPoints, 500 );
	privHandle.param( "min_quality", featureQuality, 0.01 );
	privHandle.param( "min_separation", featureSeparation, 10.0 );
	privHandle.param( "block_size", featureBlockSize, 3 );
	
	int refineWindowDim;
	privHandle.param( "window_size", refineWindowDim, 10 );
	refineWindowSize = cv::Size( refineWindowDim, refineWindowDim );
	
	privHandle.param( "use_harris", featureUseHarris, false );
	if( featureUseHarris )
	{
		privHandle.param( "harris_k", featureHarrisK, 0.04 );
	}
	
	privHandle.param( "refine_enable", refineEnable, true );
	
	if( refineEnable )
	{
		int refineMaxIters;
		double refineMinEps;
		privHandle.param( "refine_max_iters", refineMaxIters, 20 );
		privHandle.param( "refine_min_eps", refineMinEps, 0.03 );
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

} // end namespace argus
