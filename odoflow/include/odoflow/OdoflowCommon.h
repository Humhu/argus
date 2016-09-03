#pragma once

#include <opencv2/core.hpp>

#include "argus_utils/geometry/PoseSE2.h"
#include "camplex/CameraCalibration.h"

namespace argus
{

typedef cv::Point2d InterestPoint;
typedef std::vector<InterestPoint> InterestPoints;

typedef cv::Point2f InterestPointf;
typedef std::vector<InterestPointf> InterestPointsf;

std::ostream& operator<<( std::ostream& os, const InterestPoints& points );

InterestPointsf DowncastInterestPoints( const InterestPoints& in );
InterestPoints UpcastInterestPoints( const InterestPointsf& in );

// TODO Move into a utils package for open cv? Shared functionality with fiducials package
InterestPoints UndistortPoints( const InterestPoints& points, 
                                const camplex::CameraCalibration& model,
                                bool undistort, 
                                bool normalize );

InterestPoints TransformPoints( const InterestPoints& points,
                                const PoseSE2& trans );

struct FrameInterestPoints
{
	ros::Time time;
	cv::Mat frame;
	InterestPoints points;
	camplex::CameraCalibration cameraModel;

	FrameInterestPoints Undistort() const;
	FrameInterestPoints Normalize() const;
	FrameInterestPoints Unnormalize() const;
	FrameInterestPoints UndistortAndNormalize() const;
};

template<typename Scalar, typename InType>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
MatToEigen( const InType& mat )
{
	Eigen::MatrixXd ret( mat.size().height, mat.size().width);
	for( unsigned int i = 0; i < mat.size().height; i++ )
	{
		for( unsigned int j = 0; j < mat.size().width; j++ )
		{
			ret(i,j) = mat.template at<Scalar>(i,j);
		}
	}
	return ret;
}

template<typename Scalar, unsigned int height, unsigned int width>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
MatToEigen( const cv::Matx<Scalar, height, width>& mat )
{
	Eigen::MatrixXd ret( height, width );
	for( unsigned int i = 0; i < height; i++ )
	{
		for( unsigned int j = 0; j < width; j++ )
		{
			ret(i,j) = mat(i,j);
		}
	}
	return ret;
}

}
