#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "camplex/CameraCalibration.h"
#include "argus_msgs/FiducialDetection.h"
#include "argus_msgs/ImageFiducialDetections.h"

#include "argus_utils/geometry/PoseSE2.h"
#include "argus_utils/geometry/PoseSE3.h"
#include "geometry_msgs/Point.h"

#include "camplex/FiducialInfo.h"

namespace argus
{

// C++ counterpart of camplex::FiducialInfo
struct Fiducial 
{
	/*! \brief This camplex ordered points. */
	std::vector<Translation3Type> points;
	
	/*! \brief Constructs an empty fiducial. */
	Fiducial();
	
	/*! \brief Constructs from a fiducial info message. */
	Fiducial( const camplex::FiducialInfo& info );
	
	/*! \brief Returns a fiducial with transformation applied to the points. */
	Fiducial Transform( const PoseSE3& pose ) const;
		
	/*! \brief Returns a corresponding info message. */
	camplex::FiducialInfo ToMsg() const;
};

// C++ counterpart of argus_msgs::FiducialDetection
struct FiducialDetection
{
	std::string name;
	bool undistorted;
	bool normalized;
	std::vector<Translation2Type> points;

	FiducialDetection();
	FiducialDetection( const argus_msgs::FiducialDetection& msg );

	FiducialDetection Undistort( const CameraCalibration& cameraModel,
	                             bool undistort = true, 
	                             bool normalize = true );

	argus_msgs::FiducialDetection ToMsg() const;
};

// C++ counterpart of argus_msgs::ImageFiducialDetections
struct ImageFiducialDetections
{
	std::string sourceName;
	ros::Time timestamp;
	std::vector<FiducialDetection> detections;

	ImageFiducialDetections();
	ImageFiducialDetections( const argus_msgs::ImageFiducialDetections& msg );
	argus_msgs::ImageFiducialDetections ToMsg() const;
};

std::vector<cv::Point2f> PointsToCv( const std::vector<Translation2Type>& points );
std::vector<Translation2Type> CvToPoints( const std::vector<cv::Point2f>& cv );

std::vector<cv::Point3f> PointsToCv( const std::vector<Translation3Type>& points );
std::vector<Translation3Type> CvToPoints( const std::vector<cv::Point3f>& cv );

// Converts between camera (z-forward) and standard (x-forward) conventions
void CameraToStandard( const PoseSE3& cam, PoseSE3& standard );
void StandardToCamera( const PoseSE3& standard, PoseSE3& cam );

// Versions that convert to camera 2D z-forward convention and standard 3D x-forward
void CameraToStandard( const PoseSE2& cam, PoseSE3& standard );
void StandardToCamera( const PoseSE3& standard, PoseSE2& cam );

/*! \brief Simulates a fiducial detection. Ignores ROI constraints. Does not
 populate name field of detection. Returns success. */
bool ProjectDetection( const Fiducial& fiducial,
                       const CameraCalibration& cameraModel,
                       const PoseSE3& fiducialToCam,
                       FiducialDetection& detection );

/*! \brief Returns whether the detected points are entirely in the ROI. */
bool CheckDetectionROI( const FiducialDetection& det, 
                        const cv::Rect& roi );

/*! \brief Returns the min distance between a set of 2D points. Useful for estimating
 * if a fiducial detection will be valid. */
double FindMinDistance( const std::vector <argus_msgs::Point2D>& points );

PoseSE3 EstimateArrayPose( const FiducialDetection& detection,
                           const Fiducial& fiducial,
                           const PoseSE3& guess = PoseSE3() );

/*! \brief Estimates the array pose using OpenCV's solvePnP. Requires normalized and
 * undistorted detections. Assumes standard camera convention (z-forward) for 
 * imagePoints and object convention (x-forward) for input and returned poses. */
// NOTE Currently the guess functionality is broken?
PoseSE3 EstimateArrayPose( const std::vector<FiducialDetection>& detections,
                           const std::vector<Fiducial>& camplex,
                           const PoseSE3& guess = PoseSE3() );

template<typename Scalar, typename Derived>
void
EigenToMat( const Eigen::DenseBase<Derived>& in, cv::Mat& out )
{
	for( unsigned int i = 0; i < in.rows(); i++ )
	{
		for( unsigned int j = 0; j < in.cols(); j++ )
		{
			out.template at<Scalar>(i,j) = in(i,j);
		}
	}
}

template<typename Scalar, typename InType>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
MatToEigen( const InType& mat )
{
	typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> EMat;
	EMat ret( mat.size().height, mat.size().width);
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
	typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> EMat;
	EMat ret( height, width );
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
