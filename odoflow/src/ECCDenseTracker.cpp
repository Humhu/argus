#include "odoflow/OdoflowCommon.h"
#include "odoflow/ECCDenseTracker.h"

#include "argus_utils/utils/ParamUtils.h"

#include <opencv2/video/tracking.hpp>

namespace argus
{

ECCDenseTracker::ECCDenseTracker()
{
}

void ECCDenseTracker::Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph )
{
	_logMinEps.InitializeAndRead( ph, -3, "log_min_eps",
	                              "Log minimum solver tolerance" );
	_maxIters.InitializeAndRead( ph, 50, "max_iters",
	                             "Maximum solver iterations" );
	_maxIters.AddCheck<GreaterThan>( 0 );
	_minCorrelation.InitializeAndRead( ph, 0.8, "min_correlation",
	                                   "Minimum solution correlation" );
	_minCorrelation.AddCheck<GreaterThan>( 0 );
	_minCorrelation.AddCheck<LessThan>( 1.0 );

	std::string instStreamName;
	GetParam<std::string>( ph, "instruments_name", instStreamName, "ecc_tracker_instruments" );
	_instrumentsTx.InitializePushStream( instStreamName, ph, 2,
	                                     {"correlation", "runtime"}, 
										 10, "instruments" );
}

bool ECCDenseTracker::TrackImages( const cv::Mat& from,
                                   const cv::Mat& to,
                                   PoseSE3& pose,
                                   PoseSE3& rawPose )
{
	FixedMatrixType<4, 4> Hinit = pose.ToMatrix();
	FixedMatrixType<3, 2> HinitR;
	HinitR.block<2, 2>( 0, 0 ) = Hinit.block<2, 2>( 0, 0 );
	HinitR.block<2, 1>( 0, 2 ) = Hinit.block<2, 1>( 0, 3 );

	cv::Mat warp = cv::Mat::zeros( 2, 3, CV_32F );
	EigenToMat<float>( HinitR, warp );

	cv::TermCriteria termCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
	                               _maxIters, std::pow( 10, _logMinEps ) );

	// Correlation will be 0 for failures
	double corr = 0;
	ros::WallTime startTime = ros::WallTime::now();
	ros::Time startStamp = ros::Time::now();
	bool succ = false;

	try
	{
		corr = cv::findTransformECC( from, to, warp,
		                             cv::MOTION_EUCLIDEAN,
		                             termCriteria );

		succ = ( corr >= _minCorrelation );
		if( !succ )
		{
			ROS_INFO_STREAM( "Correlation " << corr << " less than min " << _minCorrelation );
		}
	}
	catch( cv::Exception )
	{
		ROS_INFO_STREAM( "ECC did not converge" );
	}

	ros::WallTime endTime = ros::WallTime::now();
	double runtime = ( endTime - startTime ).toSec();

	VectorType instruments( 2 );
	instruments << corr, runtime;
	_instrumentsTx.Publish( startStamp, instruments );

	if( succ )
	{
		Eigen::MatrixXd Wd = MatToEigen<float>( warp ).cast<double>();

		FixedMatrixType<4, 4> H = FixedMatrixType<4, 4>::Identity();
		H.block<2, 2>( 1, 1 ) = Wd.block<2, 2>( 0, 0 );
		H( 1, 3 ) = -Wd( 0, 2 );
		H( 2, 3 ) = -Wd( 1, 2 );
		pose = PoseSE3( H ).Inverse();

		H.block<2, 1>( 0, 3 ) = Wd.block<2, 1>( 0, 2 );
		rawPose = PoseSE3( H );
	}

	return succ;
}
}