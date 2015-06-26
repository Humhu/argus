#ifndef _V4L2D_CAMERACALIB_H_
#define _V4L2D_CAMERACALIB_H_

#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>
#include <memory>
#include <opencv2/core.hpp>

namespace v4l2_cam
{
	
	/*! \brief Wraps a ROS camera calibration model to allow scaling to different resolutions. */
	class CameraCalibration 
	{
	public:

		typedef std::shared_ptr<CameraCalibration> Ptr;
		
		CameraCalibration( const std::string& name, const sensor_msgs::CameraInfo& info );
		CameraCalibration( const std::string& calibPath );
		
		std::string GetName() const;
		
		cv::Size GetScale() const;
		void SetScale( const cv::Size& resolution );
		
		sensor_msgs::CameraInfo GetInfo() const;
		
		double GetFx() const;
		double GetFy() const;
		double GetCx() const;
		double GetCy() const;
		
		void SetFx( double fx );
		void SetFy( double fy );
		void SetCx( double cx );
		void SetCy( double cy );
		
	private:
		
		std::string cameraName;
		sensor_msgs::CameraInfo cameraInfo;
		cv::Size currentScale;
		sensor_msgs::CameraInfo origInfo;
		
	};
	
}

#endif

