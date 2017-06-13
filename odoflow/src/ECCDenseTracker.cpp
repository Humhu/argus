#include "odoflow/OdoflowCommon.h"
#include "odoflow/ECCDenseTracker.h"

#include "camplex/FiducialCommon.h"

#include <opencv2/video/tracking.hpp>

namespace argus
{

ECCDenseTracker::ECCDenseTracker(ros::NodeHandle &nh, ros::NodeHandle &ph)
{
    _logMinEps.InitializeAndRead(ph, -3, "log_min_eps",
                                 "Log minimum solver tolerance");
    _maxIters.InitializeAndRead(ph, 50, "max_iters",
                                "Maximum solver iterations");
    _maxIters.AddCheck<GreaterThan>( 0 );
    _minCorrelation.InitializeAndRead(ph, 0.8, "min_correlation",
                                      "Minimum solution correlation");
    _minCorrelation.AddCheck<GreaterThan>( 0 );
    _minCorrelation.AddCheck<LessThan>( 1.0 );
}

bool ECCDenseTracker::TrackImages(const cv::Mat &to,
                                  const cv::Mat &from,
                                  PoseSE2& pose)
{
    FixedMatrixType<3,3> Hinit = pose.ToMatrix();
    FixedMatrixType<2,3> HinitR = Hinit.block<2,3>(0,0);

    cv::Mat warp = cv::Mat::zeros(2, 3, CV_32F);
    EigenToMat<float>( HinitR, warp );
    
    cv::TermCriteria termCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                  _maxIters, std::pow(10, _logMinEps));

    try
    {
        double ecc = cv::findTransformECC(from, to, warp,
                                          cv::MOTION_EUCLIDEAN,
                                          termCriteria);
        if( ecc < _minCorrelation )
        {
            ROS_INFO_STREAM( "Correlation " << ecc << " less than min " << _minCorrelation );
            return false;
        }
    }
    catch (cv::Exception)
    {
        return false;
    }

    Eigen::MatrixXd Wd = MatToEigen<float>(warp).cast<double>();
	
	PoseSE2::Rotation R(0);
	R.fromRotationMatrix( Wd.topLeftCorner<2,2>() );
	Translation2Type t( Wd(0,2), Wd(1,2) );

	pose = PoseSE2(t, R);
    return true;
}
}