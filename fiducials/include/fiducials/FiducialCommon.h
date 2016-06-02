#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "camplex/CameraCalibration.h"
#include "argus_msgs/FiducialDetection.h"
#include "argus_msgs/ImageFiducialDetections.h"

#include "argus_utils/geometry/PoseSE3.h"

#include "fiducials/Fiducial.h"
#include "fiducials/FiducialInfo.h"

namespace argus
{

/*! \brief Convert ROS message types to OpenCV points. */
std::vector <cv::Point2f> MsgToPoints( const std::vector <argus_msgs::Point2D>& msg );
std::vector <cv::Point3f> MsgToPoints( const std::vector <geometry_msgs::Point>& msg );

/*! \brief Convert OpenCV points to ROS message types. */
std::vector <argus_msgs::Point2D> PointsToMsg( const std::vector <cv::Point2f>& points );
std::vector <geometry_msgs::Point> PointsToMsg( const std::vector <cv::Point3f>& points );

/*! \brief Convert OpenCV points to Eigen matrix. */
Eigen::Matrix <double, 2, Eigen::Dynamic> PointsToMatrix( const std::vector <cv::Point2f>& points );
Eigen::Matrix <double, 3, Eigen::Dynamic> PointsToMatrix( const std::vector <cv::Point3f>& points );

/*! \brief Convert Eigen matrix to OpenCV points. */
std::vector <cv::Point2f> MatrixToPoints( const Eigen::Matrix <double, 2, Eigen::Dynamic>& mat );
std::vector <cv::Point3f> MatrixToPoints( const Eigen::Matrix <double, 3, Eigen::Dynamic>& mat );

/*! \brief Convert ROS message types to Eigen matrix. */
std::vector <argus_msgs::Point2D> MatrixToMsg( const Eigen::Matrix <double, 2, Eigen::Dynamic>& mat );
std::vector <geometry_msgs::Point> MatrixToMsg( const Eigen::Matrix <double, 3, Eigen::Dynamic>& mat );

/*! \brief Convert Eigen matrices to ROS message types. */
Eigen::Matrix <double, 2, Eigen::Dynamic> MsgToMatrix( const std::vector <argus_msgs::Point2D>& msg );
Eigen::Matrix <double, 3, Eigen::Dynamic> MsgToMatrix( const std::vector <geometry_msgs::Point>& msg );

/*! \brief Undistort and normalize fiducial detections in-place. Assumes all detections
 * have the same undistortion/normalization status. */
bool UndistortDetections( const std::vector <argus_msgs::FiducialDetection>& detections,
                          const camplex::CameraCalibration& cameraModel, 
                          bool undistort, bool normalize,
                          std::vector< argus_msgs::FiducialDetection >& undistorted );

/*! \brief Simulates a fiducial detection. Ignores ROI constraints. Does not
 populate name field of detection. Returns success. */
bool ProjectDetection( const Fiducial& fiducial,
                       const camplex::CameraCalibration& cameraModel,
                       const PoseSE3& fiducialToCam,
                       argus_msgs::FiducialDetection& detection );

/*! \brief Returns whether the detected points are entirely in the ROI. */
bool CheckDetectionROI( const argus_msgs::FiducialDetection& det, 
                        const cv::Rect& roi );

/*! \brief Returns the min distance between a set of 2D points. Useful for estimating
 * if a fiducial detection will be valid. */
double FindMinDistance( const std::vector <argus_msgs::Point2D>& points );

}
