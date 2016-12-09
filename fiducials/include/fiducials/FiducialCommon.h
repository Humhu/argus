#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "camplex/CameraCalibration.h"
#include "argus_msgs/FiducialDetection.h"
#include "argus_msgs/ImageFiducialDetections.h"

#include "argus_utils/geometry/PoseSE3.h"
#include "geometry_msgs/Point.h"

#include "fiducials/FiducialInfo.h"

namespace argus
{

// C++ counterpart of fiducials::FiducialInfo
struct Fiducial 
{
	/*! \brief This fiducials ordered points. */
	std::vector <Translation3Type> points;
	
	/*! \brief Constructs an empty fiducial. */
	Fiducial();
	
	/*! \brief Constructs from a fiducial info message. */
	Fiducial( const fiducials::FiducialInfo& info );
	
	/*! \brief Returns a fiducial with transformation applied to the points. */
	Fiducial Transform( const PoseSE3& pose ) const;
		
	/*! \brief Returns a corresponding info message. */
	fiducials::FiducialInfo ToMsg() const;
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

	FiducialDetection Undistort( const camplex::CameraCalibration& cameraModel,
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

/*! \brief Simulates a fiducial detection. Ignores ROI constraints. Does not
 populate name field of detection. Returns success. */
bool ProjectDetection( const Fiducial& fiducial,
                       const camplex::CameraCalibration& cameraModel,
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
                           const std::vector<Fiducial>& fiducials,
                           const PoseSE3& guess = PoseSE3() );

}
