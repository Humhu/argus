#include "odoflow/CornerPointDetector.h"

namespace odoflow
{

	CornerPointDetector::CornerPointDetector() {}
	
	InterestPoints CornerPointDetector::FindInterestPoints( const cv::Mat& image )
	{
		InterestPoints points;
		cv::goodFeaturesToTrack( image, points, featureMaxPoints, featureQuality, featureSeparation, 
								 cv::noArray(), featureBlockSize, featureUseHarris, featureHarrisK );
		return points;
	}
	
	void CornerPointDetector::SetMaxPoints( int maxPoints )
	{
		featureMaxPoints = maxPoints;
	}
	
	void CornerPointDetector::SetMinQuality( double quality )
	{
		featureQuality = quality;
	}
	
	void CornerPointDetector::SetMinSeparation( double separation )
	{
		featureSeparation = separation;
	}
	
	void CornerPointDetector::SetQualityWindowSize( int winSize )
	{
		featureBlockSize = winSize;
	}
	
	void CornerPointDetector::SetHarrisMode( bool enable )
	{
		featureUseHarris = enable;
	}
	
	void CornerPointDetector::SetHarrisK( double k )
	{
		featureHarrisK = k;
	}
	
	void CornerPointDetector::SetRefinementMode( bool enable )
	{
		refineEnable = enable;
	}
	
	void CornerPointDetector::SetRefinementWindow( int width, int height )
	{
		refineWindowSize = cv::Size( width, height );
	}
	
	void CornerPointDetector::SetRefinementCriteria( int maxIters, double epsilon )
	{
		refineTermCriteria = cv::TermCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS, maxIters, epsilon );
	}
	
}
