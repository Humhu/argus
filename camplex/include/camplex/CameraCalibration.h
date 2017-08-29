#pragma once

#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>
#include <memory>
#include <opencv2/core.hpp>

#include <argus_utils/utils/LinalgTypes.h>

#include <iostream>

namespace argus
{

/*! \brief Wraps a ROS camera calibration model to allow scaling to different resolutions. */
class CameraCalibration
{
public:

	typedef std::shared_ptr<CameraCalibration> Ptr;

	/*! \brief Initializes a model with identity camera matrix, no distortion coefficients,
	   and unity scale. */
	CameraCalibration();

	/*! \brief Creates a model from the specified ROS message. */
	CameraCalibration( const std::string& name, const sensor_msgs::CameraInfo& info );

	/*! \brief Attempts to read a model from a calibration file. */
	CameraCalibration( const std::string& calibPath );

	/*! \brief Creates a model with specified parameters. Used for adapting OpenCV calibration routines. */
	CameraCalibration( const std::string& name, const cv::Size& scale,
	                   const cv::Matx33d& intrinsics, const cv::Mat& distortion );

	std::string GetName() const;

	cv::Size GetScale() const;
	void SetScale( const cv::Size& resolution ); // TODO Rename
	cv::Rect GetRoi() const;

	/*! /brief Populates the calibration from a ROS message. Resets the scale
	 * to unity. */
	void ParseInfo( const sensor_msgs::CameraInfo& info );
	sensor_msgs::CameraInfo GetInfo() const;

	double GetFx() const;
	double GetFy() const;
	double GetCx() const;
	double GetCy() const;

	const cv::Mat& GetDistortionCoeffs() const;
	const cv::Matx33d& GetIntrinsicMatrix() const;

private:

	std::string _cameraName;
	cv::Matx33d _currentCameraMatrix;
	cv::Size _currentScale;

	cv::Matx33d _origCameraMatrix;
	cv::Size _origScale;

	FixedMatrixType<3,3> _R;
	FixedMatrixType<3,4> _P;

	// NOTE Distortion coefficients are normalized and thus scale-invariant
	cv::Mat _distortionCoeffs;

	double& GetFx( sensor_msgs::CameraInfo& info );
	const double& GetFx( const sensor_msgs::CameraInfo& info ) const;
	double& GetFy( sensor_msgs::CameraInfo& info );
	const double& GetFy( const sensor_msgs::CameraInfo& info ) const;
	double& GetCx( sensor_msgs::CameraInfo& info );
	const double& GetCx( const sensor_msgs::CameraInfo& info ) const;
	double& GetCy( sensor_msgs::CameraInfo& info );
	const double& GetCy( const sensor_msgs::CameraInfo& info ) const;

	void ReadDistortionCoeffs( const sensor_msgs::CameraInfo& info );
	void ReadIntrinsicMatrix( const sensor_msgs::CameraInfo& info );
};

std::ostream& operator<<( std::ostream& os, const CameraCalibration& calib );

}
