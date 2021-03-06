#include "odoflow/OdoflowCommon.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

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

InterestPoints ProcessPoints( const InterestPoints& points,
                              const CameraCalibration& model,
                              bool undistort,
                              bool normalize )
{
	// Perform undistortion
	cv::Mat distortionCoeffs;
	cv::Matx33d cameraMatrix;

	if( undistort ) { distortionCoeffs = model.GetDistortionCoeffs(); }
	cameraMatrix = normalize ? cv::Matx33d::eye() : model.GetIntrinsicMatrix();

	InterestPoints undistorted;
	cv::undistortPoints( points, undistorted, model.GetIntrinsicMatrix(),
	                     distortionCoeffs, cv::noArray(), cameraMatrix );
	return undistorted;
}

InterestPoints UnprocessPoints( const InterestPoints& points,
                                const CameraCalibration& model,
                                bool distort,
                                bool unnormalize )
{
	std::vector<cv::Point3d> points3;
	convertPointsToHomogeneous( points, points3 );

	InterestPoints distPoints;
	cv::Mat distortionCoeffs;
	cv::Matx33d cameraMatrix;

	if( distort ) { distortionCoeffs = model.GetDistortionCoeffs(); }
	cameraMatrix = unnormalize ? model.GetIntrinsicMatrix() : cv::Matx33d::eye();

	projectPoints( points3,
	               cv::Vec3f( 0, 0, 0 ),
	               cv::Vec3f( 0, 0, 0 ),
	               cameraMatrix,
	               distortionCoeffs,
	               distPoints );
	return distPoints;
}

InterestPoints UndistortAndNormalizePoints( const InterestPoints& points,
                                            const CameraCalibration& model )
{
	return ProcessPoints( points, model, true, true );
}

InterestPoints UndistortPoints( const InterestPoints& points,
                                const CameraCalibration& model )
{
	return ProcessPoints( points, model, true, false );
}

InterestPoints NormalizePoints( const InterestPoints& points,
                                const CameraCalibration& model )
{
	return ProcessPoints( points, model, false, true );
}

InterestPoints DistortPoints( const InterestPoints& points,
                              const CameraCalibration& model )
{
	return UnprocessPoints( points, model, true, false );
}

InterestPoints UnnormalizePoints( const InterestPoints& points,
                                  const CameraCalibration& model )
{
	return UnprocessPoints( points, model, false, true );
}

InterestPoints DistortAndUnnormalizePoints( const InterestPoints& points,
                                            const CameraCalibration& model )
{
	return UnprocessPoints( points, model, true, true );
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
		pts( 0, i ) = points[i].x;
		pts( 1, i ) = points[i].y;
	}

	pts = trans.ToTransform() * pts;
	InterestPoints transformed;
	transformed.reserve( points.size() );
	for( unsigned int i = 0; i < points.size(); ++i )
	{
		transformed.emplace_back( pts( 0, i ), pts( 1, i ) );
	}
	return transformed;
}
}
