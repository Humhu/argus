#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include "argus_msgs/FiducialDetection.h"
#include "argus_msgs/ImageFiducialDetections.h"

#include "fiducial_array/FiducialInfo.h"

namespace fiducial_array
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
bool UndistortDetections( const std::vector< argus_msgs::FiducialDetection >& detections,
                          const image_geometry::PinholeCameraModel& cameraModel, 
                          bool undistort, bool normalize,
                          std::vector< argus_msgs::FiducialDetection >& undistorted );

}
