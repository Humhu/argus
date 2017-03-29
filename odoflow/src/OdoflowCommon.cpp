#include "odoflow/OdoflowCommon.h"
#include <opencv2/imgproc.hpp>

namespace argus
{

std::ostream& operator<<( std::ostream& os, const InterestPoints& points )
{
	os << "Interest points: " << std::endl;
	for( unsigned int i = 0; i < points.size(); ++i )
	{
		os << "\t(x: " << points[i].x << ", y: " << points[i].y << ")" << std::endl;
	}
	return os;
}

InterestPointsf DowncastInterestPoints( const InterestPoints& in )
{
	InterestPointsf out( in.size() );
	for( unsigned int i = 0; i < in.size(); i++ )
	{
		out[i] = cv::Point2f( in[i].x, in[i].y );
	}
	return out;
}

InterestPoints UpcastInterestPoints( const InterestPointsf& in )
{
	InterestPoints out( in.size() );
	for( unsigned int i = 0; i < in.size(); i++ )
	{
		out[i] = cv::Point2d( in[i].x, in[i].y );
	}
	return out;
}


InterestPoints UndistortPoints( const InterestPoints& points, 
                                const CameraCalibration& model,
                                bool undistort, 
                                bool normalize )
{
	if( points.empty() )
	{
		throw std::invalid_argument( "UndistortPoints: Empty points given." );
	}
	if( !std::isfinite( model.GetFx() ) )
	{
		throw std::invalid_argument( "UndistortPoints: Invalid camera model." );
	}

	// Perform undistortion
	cv::Mat distortionCoeffs;
	cv::Matx33d outputCameraMatrix;
	
	if( undistort ) { distortionCoeffs = model.GetDistortionCoeffs(); }
	outputCameraMatrix = normalize ? cv::Matx33d::eye() : model.GetIntrinsicMatrix();
	
	InterestPoints undistorted;
	cv::undistortPoints( points, undistorted, model.GetIntrinsicMatrix(),
	                     distortionCoeffs, cv::noArray(), outputCameraMatrix );
	return undistorted;
}

InterestPoints TransformPoints( const InterestPoints& points,
                                const PoseSE2& trans )
{
	if( points.empty() )
	{
		throw std::invalid_argument( "TransformPoints: Empty points given." );
	}

	Eigen::Matrix<double, 2, Eigen::Dynamic> pts( 2, points.size() );
	for( unsigned int i = 0; i < points.size(); ++i )
	{
		pts(0,i) = points[i].x;
		pts(1,i) = points[i].y;
	}

	pts = trans.ToTransform() * pts;
	InterestPoints transformed;
	transformed.reserve( points.size() );
	for( unsigned int i = 0; i < points.size(); ++i )
	{
		transformed.emplace_back( pts(0,i), pts(1,i) );
	}
	return transformed;
}

FrameInterestPoints
FrameInterestPoints::Undistort() const
{
	FrameInterestPoints ret( *this );
	cv::undistortPoints( points, 
	                     ret.points,
	                     cameraModel.GetIntrinsicMatrix(),
	                     cameraModel.GetDistortionCoeffs(), 
	                     cv::noArray(), 
	                     cameraModel.GetIntrinsicMatrix() );
	return ret;
}

FrameInterestPoints 
FrameInterestPoints::Normalize() const
{
	FrameInterestPoints ret( *this );
	cv::undistortPoints( points, 
	                     ret.points, 
	                     cameraModel.GetIntrinsicMatrix(),
	                     cv::Mat(), 
	                     cv::noArray(), 
	                     cv::Matx33d::eye() );
	return ret;
}

FrameInterestPoints 
FrameInterestPoints::Unnormalize() const
{
	FrameInterestPoints ret( *this );
	cv::undistortPoints( points, 
	                     ret.points, 
	                     cv::Matx33d::eye(),
	                     cv::Mat(), 
	                     cv::noArray(), 
	                     cameraModel.GetIntrinsicMatrix() );
	return ret;
}

FrameInterestPoints 
FrameInterestPoints::UndistortAndNormalize() const
{
	FrameInterestPoints ret( *this );
	cv::undistortPoints( points, 
	                     ret.points, 
	                     cameraModel.GetIntrinsicMatrix(),
	                     cameraModel.GetDistortionCoeffs(), 
	                     cv::noArray(), 
	                     cameraModel.GetIntrinsicMatrix() );
	return ret;
}

}
