#include "odoflow/OdoflowCommon.h"
#include "odoflow/ECCDenseTracker.h"

#include "camplex/FiducialCommon.h"

#include <opencv2/video/tracking.hpp>

namespace argus
{
ECCDenseTracker::ECCDenseTracker( ros::NodeHandle& nh, ros::NodeHandle& ph )
{
	_logMinEps.InitializeAndRead( ph, -3, "log_min_eps",
	                              "Log10 minimum solver tolerance" );
	_maxIters.InitializeAndRead( ph, 50, "max_iters",
	                             "Maximum solver iterations" );
	_maxIters.AddCheck<GreaterThan>( 0 );
	_logMinCorrelation.InitializeAndRead( ph, -2.0, "log_min_correlation",
	                                      "log10 of 1.0 - minimum solution correlation" );
	_logMinCorrelation.AddCheck<LessThan>( std::log10( 1.0 ) );
}

bool ECCDenseTracker::TrackImages( const cv::Mat& to,
                                   const cv::Mat& from,
                                   PoseSE2& pose )
{
	FixedMatrixType<3, 3> Hinit = pose.ToMatrix();
	FixedMatrixType<2, 3> HinitR = Hinit.block<2, 3>( 0, 0 );
	cv::Mat warp = cv::Mat::zeros( 2, 3, CV_32F );
	EigenToMat<float>( HinitR, warp );

	cv::TermCriteria termCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
	                               _maxIters,
	                               std::pow( 10, _logMinEps ) );

	try
	{
		double ecc = cv::findTransformECC( from, to, warp,
		                                   cv::MOTION_EUCLIDEAN,
		                                   termCriteria );
		double minECC = 1.0 - std::pow( 10, _logMinCorrelation );
		if( ecc < minECC )
		{
			ROS_INFO_STREAM( "Correlation " << ecc << " less than min " << minECC );
			return false;
		}
	}
    // NOTE findTransformECC throws an exception for some failure modes
	catch( cv::Exception )
	{
		return false;
	}

	Eigen::MatrixXd Wd = MatToEigen<float>( warp ).cast<double>();
	PoseSE2::Rotation R( 0 );
	R.fromRotationMatrix( Wd.topLeftCorner<2, 2>() );
	Translation2Type t( Wd( 0, 2 ), Wd( 1, 2 ) );
	pose = PoseSE2( t, R );
	return true;
}
}
