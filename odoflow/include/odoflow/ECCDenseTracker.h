#pragma once

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include "paraset/ParameterManager.hpp"

#include "argus_utils/geometry/PoseSE3.h"

namespace argus
{

/*! \brief Uses OpenCV's findTransformECC to match two images using
direct intensity values. 
*/
class ECCDenseTracker
{
public:

    ECCDenseTracker( ros::NodeHandle& nh, ros::NodeHandle& ph );

    bool TrackImages( const cv::Mat& from, const cv::Mat& to,
                         PoseSE3& pose );

private:

    NumericParam _logMinEps;
    NumericParam _maxIters;
    NumericParam _minCorrelation;

};

}