#pragma once

#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>
#include <memory>
#include <opencv2/core.hpp>

#include <iostream>

namespace camplex
{
	
/*! \brief Wraps a ROS camera calibration model to allow scaling to different resolutions. */
class CameraCalibration 
{
public:

	typedef std::shared_ptr<CameraCalibration> Ptr;
	
	CameraCalibration();
	CameraCalibration( const std::string& name, const sensor_msgs::CameraInfo& info );
	CameraCalibration( const std::string& calibPath );
	CameraCalibration( const std::string& name, double fx, double fy, 
					   double px, double py, double w, double h );
	
	std::string GetName() const;
	
	cv::Size GetScale() const;
	void SetScale( const cv::Size& resolution ); // TODO Rename
	cv::Rect GetRoi() const;
	
	sensor_msgs::CameraInfo GetInfo() const;
	
	double GetFx() const;
	double GetFy() const;
	double GetCx() const;
	double GetCy() const;
	
	const cv::Mat& GetDistortionCoeffs() const;
	const cv::Matx33d& GetIntrinsicMatrix() const;
	
private:
	
	std::string cameraName;
	sensor_msgs::CameraInfo cameraInfo;
	cv::Size currentScale;
	sensor_msgs::CameraInfo origInfo;
	
	cv::Mat distortionCoeffs;
	cv::Matx33d intrinsicMatrix;
	
	double& GetFx( sensor_msgs::CameraInfo& info );
	const double& GetFx( const sensor_msgs::CameraInfo& info  ) const;
	double& GetFy(sensor_msgs::CameraInfo& info );
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
