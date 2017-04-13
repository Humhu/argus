#include "camplex/CameraCalibration.h"

#define AR_RATIO_MIN ( 0.99 )
#define AR_RATIO_MAX ( 1.01 )

namespace argus
{

CameraCalibration::CameraCalibration()
{
	_cameraName = "";
	_origCameraMatrix = cv::Matx33d::eye();
	_origScale = cv::Size( 1.0, 1.0 );
	SetScale( _origScale );
}

CameraCalibration::CameraCalibration( const std::string& name,
                                      const sensor_msgs::CameraInfo& info )
	: _cameraName( name )
{
	ParseInfo( info );
	SetScale( _origScale );
}

CameraCalibration::CameraCalibration( const std::string& calibPath )
{
	sensor_msgs::CameraInfo info;
	if( !camera_calibration_parsers::readCalibration( calibPath, _cameraName, info ) )
	{
		throw std::runtime_error( "Could not read calibration at: " + calibPath );
	}
	ParseInfo( info );
	SetScale( _origScale );
}

CameraCalibration::CameraCalibration( const std::string& name,
                                      const cv::Size& scale,
                                      const cv::Matx33d& intrinsics,
                                      const cv::Mat& distortion )
	: _cameraName( name ), _origScale( scale ), _origCameraMatrix( intrinsics )
{
	distortion.copyTo( _distortionCoeffs );
	SetScale( _origScale );
}


std::string CameraCalibration::GetName() const
{
	return _cameraName;
}

cv::Size CameraCalibration::GetScale() const
{
	return _currentScale;
}

cv::Rect CameraCalibration::GetRoi() const
{
	return cv::Rect( 0, 0, _currentScale.width, _currentScale.height );
}

void CameraCalibration::SetScale( const cv::Size& scale )
{
	double aspectRatio = ( (double) scale.width ) / scale.height;
	double origAspectRatio = ( (double) _origScale.width ) / _origScale.height;
	double arRatio = aspectRatio / origAspectRatio;
	if( arRatio > AR_RATIO_MAX || arRatio < AR_RATIO_MIN )
	{
		throw std::invalid_argument( "Specified aspect ratio does not match original." );
	}

	double xRatio = ( (double) scale.width ) / _origScale.width;
	double yRatio = ( (double) scale.height ) / _origScale.height;
	double ratio = ( xRatio + yRatio ) / 2;

	_currentCameraMatrix = ratio * _origCameraMatrix;
	_currentCameraMatrix( 2, 2 ) = 1.0;
	_currentScale = scale;
}

void CameraCalibration::ParseInfo( const sensor_msgs::CameraInfo& info )
{
	// Read intrinsics matrix row-major
	unsigned int ind = 0;
	for( unsigned int i = 0; i < 3; ++i )
	{
		for( unsigned int j = 0; j < 3; ++j )
		{
			_origCameraMatrix( i, j ) = info.K[ind];
			++ind;
		}
	}

	// Read distortion coefficients
	_distortionCoeffs = cv::Mat( info.D.size(), 1, CV_64F );
	for( unsigned int i = 0; i < info.D.size(); i++ )
	{
		_distortionCoeffs.at<double>( i ) = info.D[i];
	}

	// Read scale
	_origScale.height = info.height;
	_origScale.width = info.width;

	SetScale( _origScale );
}

sensor_msgs::CameraInfo CameraCalibration::GetInfo() const
{
	sensor_msgs::CameraInfo msg;
	msg.header.frame_id = _cameraName;
	msg.height = _currentScale.height;
	msg.width = _currentScale.width;
	msg.distortion_model = "plumb_bob";
	for( unsigned int i = 0; i < _distortionCoeffs.total(); i++ )
	{
		msg.D.push_back( _distortionCoeffs.at<double>( i ) );
	}
	unsigned int ind = 0;
	for( unsigned int i = 0; i < 3; ++i )
	{
		for( unsigned int j = 0; j < 3; ++j )
		{
			msg.K[ind] = _currentCameraMatrix( i, j );
			++ind;
		}
	}
	// TODO Populate R, T, and P
	return msg;
}

double CameraCalibration::GetFx() const
{
	return _currentCameraMatrix( 0, 0 );
}

double CameraCalibration::GetFy() const
{
	return _currentCameraMatrix( 1, 1 );
}

double CameraCalibration::GetCx() const
{
	return _currentCameraMatrix( 0, 2 );
}

double CameraCalibration::GetCy() const
{
	return _currentCameraMatrix( 1, 2 );
}

const cv::Mat& CameraCalibration::GetDistortionCoeffs() const
{
	return _distortionCoeffs;
}

const cv::Matx33d& CameraCalibration::GetIntrinsicMatrix() const
{
	return _currentCameraMatrix;
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
		os << " " << distCoeffs.at<double>( i );
	}
	os << std::endl;
	return os;
}


}
