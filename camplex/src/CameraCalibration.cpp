#include "camplex/CameraCalibration.h"

namespace camplex
{

CameraCalibration::CameraCalibration() {}

CameraCalibration::CameraCalibration( const std::string& name, const sensor_msgs::CameraInfo& info )
: cameraName( name )
{
	origInfo = info;
	cameraInfo = origInfo;
	
	ReadDistortionCoeffs( info );
	
	SetScale( cv::Size( cameraInfo.width, cameraInfo.height ) );
}

CameraCalibration::CameraCalibration( const std::string& calibPath )
{
	if( !camera_calibration_parsers::readCalibration( calibPath, cameraName, origInfo ) )
	{
		throw std::runtime_error( "Could not read calibration at: " + calibPath );
	}
	cameraInfo = origInfo;
	
	ReadDistortionCoeffs( origInfo );
	
	SetScale( cv::Size( cameraInfo.width, cameraInfo.height ) );
}

CameraCalibration::CameraCalibration( const std::string& name, double fx, double fy, 
                                      double px, double py, double w, double h )
	: cameraName( name )
{
	GetFx( origInfo ) = fx;
	GetFy( origInfo ) = fy;
	GetCx( origInfo ) = px;
	GetCy( origInfo ) = py;
	origInfo.width = w;
	origInfo.height = h;
	cameraInfo = origInfo;
	
	ReadDistortionCoeffs( origInfo );
	
	SetScale( cv::Size( w, h ) );
}

std::string CameraCalibration::GetName() const
{
	return cameraName;
}

cv::Size CameraCalibration::GetScale() const
{
	return currentScale;
}

cv::Rect CameraCalibration::GetRoi() const
{
	return cv::Rect( 0, 0, currentScale.width, currentScale.height );
}

void CameraCalibration::SetScale( const cv::Size& scale )
{
	double xRatio = ( (double) scale.width ) / origInfo.width;
	double yRatio = ( (double) scale.height ) / origInfo.height;
	
	cameraInfo.width = scale.width;
	cameraInfo.height = scale.height;
	GetFx( cameraInfo ) = GetFx( origInfo ) * xRatio;
	GetCx( cameraInfo ) = GetCx( origInfo ) * xRatio;
	GetFy( cameraInfo ) = GetFy( origInfo ) * yRatio;
	GetCy( cameraInfo ) = GetCy( origInfo ) * yRatio;
	
	currentScale = scale;
	
	ReadIntrinsicMatrix( cameraInfo );
}

sensor_msgs::CameraInfo CameraCalibration::GetInfo() const
{
	return cameraInfo;
}

double CameraCalibration::GetFx() const
{
	return GetFx( cameraInfo );
}

double CameraCalibration::GetFy() const
{
	return GetFy( cameraInfo );
}

double CameraCalibration::GetCx() const
{
	return GetCx( cameraInfo );
}

double CameraCalibration::GetCy() const
{
	return GetCy( cameraInfo );
}

double& CameraCalibration::GetFx( sensor_msgs::CameraInfo& info ) 
{ 
	return info.K[0]; 
}

const double& CameraCalibration::GetFx( const sensor_msgs::CameraInfo& info ) const 
{ 
	return info.K[0]; 
}

double& CameraCalibration::GetFy( sensor_msgs::CameraInfo& info ) 
{ 
	return info.K[4]; 
}

const double& CameraCalibration::GetFy( const sensor_msgs::CameraInfo& info ) const 
{ 
	return info.K[4]; 
}

double& CameraCalibration::GetCx( sensor_msgs::CameraInfo& info ) 
{ 
	return info.K[2]; 
}

const double& CameraCalibration::GetCx( const sensor_msgs::CameraInfo& info ) const 
{ 
	return info.K[2]; 
}

double& CameraCalibration::GetCy( sensor_msgs::CameraInfo& info ) 
{ 
	return info.K[5]; 
}

const double& CameraCalibration::GetCy( const sensor_msgs::CameraInfo& info ) const 
{ 
	return info.K[5]; 
}

const cv::Mat& CameraCalibration::GetDistortionCoeffs() const 
{
	return distortionCoeffs;
}

const cv::Matx33d& CameraCalibration::GetIntrinsicMatrix() const
{
	return intrinsicMatrix;
}

void CameraCalibration::ReadDistortionCoeffs( const sensor_msgs::CameraInfo& info ) 
{
	distortionCoeffs = cv::Mat( info.D.size(), 1, CV_64F );
	for( unsigned int i = 0; i < info.D.size(); i++ )
	{
		distortionCoeffs.at<double>(i) = info.D[i];
	}
}

void CameraCalibration::ReadIntrinsicMatrix( const sensor_msgs::CameraInfo& info )
{
	intrinsicMatrix = cv::Matx33d::eye();
	intrinsicMatrix(0,0) = GetFx( info );
	intrinsicMatrix(1,1) = GetFy( info );
	intrinsicMatrix(0,2) = GetCx( info );
	intrinsicMatrix(1,2) = GetCy( info );
}

std::ostream& operator<<( std::ostream& os, const CameraCalibration& calib )
{
	os << "Camera calibration: " << std::endl;
	os << "\tFocal: (fx: " << calib.GetFx() << ", fy: " << calib.GetFy() << ")" << std::endl;
	os << "\tPrincipal: (cx: " << calib.GetCx() << ", cy: " << calib.GetCy() << ")" << std::endl;
	
	os << "\tDistortion:";
	cv::Mat distCoeffs = calib.GetDistortionCoeffs();
	for( unsigned int i = 0; i < distCoeffs.total(); i++ )
	{
		os << " " << distCoeffs.at<double>(i);
	}
	os << std::endl;
	return os;
}


}
