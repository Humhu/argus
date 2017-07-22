#pragma once

#include "opencv2/calib3d.hpp"

namespace argus
{
class StereoDisparityProcessor
{
public:

	StereoDisparityProcessor();
	virtual ~StereoDisparityProcessor();

	virtual void Process( const cv::Mat& left, const cv::Mat& right,
	                      cv::Mat& disparity ) = 0;
};
}