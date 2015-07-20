#include "v4l2_cam/CameraCalibration.h"

namespace v4l2_cam
{

	CameraCalibration::CameraCalibration( const std::string& name, const sensor_msgs::CameraInfo& info )
		: cameraName( name )
	{
		origInfo = info;
		cameraInfo = origInfo;
	}
	
	CameraCalibration::CameraCalibration( const std::string& calibPath )
	{
		camera_calibration_parsers::readCalibration( calibPath, cameraName, origInfo );
		cameraInfo = origInfo;
	}
	
	std::string CameraCalibration::GetName() const
	{
		return cameraName;
	}
	
	cv::Size CameraCalibration::GetScale() const
	{
		return currentScale;
	}
	
	void CameraCalibration::SetScale( const cv::Size& scale )
	{
		double xRatio = ( (double) scale.width ) / origInfo.width;
		double yRatio = ( (double) scale.height ) / origInfo.height;
		
		cameraInfo.width = scale.width;
		cameraInfo.height = scale.height;
		cameraInfo.K[0] = origInfo.K[0]*xRatio; // fx
		cameraInfo.K[2] = origInfo.K[2]*xRatio; // cx
		cameraInfo.K[4] = origInfo.K[4]*yRatio; // fy
		cameraInfo.K[5] = origInfo.K[5]*yRatio; // cy
		
		cameraInfo.P[0] = origInfo.P[0]*xRatio;
		cameraInfo.P[2] = origInfo.P[2]*xRatio;
		cameraInfo.P[5] = origInfo.P[5]*yRatio;
		cameraInfo.P[6] = origInfo.P[6]*yRatio;
		currentScale = scale;
	}
	
	sensor_msgs::CameraInfo CameraCalibration::GetInfo() const
	{
		return cameraInfo;
	}
	
	double CameraCalibration::GetFx() const
	{
		return cameraInfo.K[0];
	}
	
	double CameraCalibration::GetFy() const
	{
		return cameraInfo.K[4];
	}
	
	double CameraCalibration::GetCx() const
	{
		return cameraInfo.K[2];
	}
	
	double CameraCalibration::GetCy() const
	{
		return cameraInfo.K[5];
	}
	
	void CameraCalibration::SetFx( double fx )
	{
		cameraInfo.K[0] = fx;
	}
	
	void CameraCalibration::SetFy( double fy )
	{
		cameraInfo.K[4] = fy;
	}
	
	void CameraCalibration::SetCx( double cx )
	{
		cameraInfo.K[2] = cx;
	}
	
	void CameraCalibration::SetCy( double cy )
	{
		cameraInfo.K[5] = cy;
	}
	
}