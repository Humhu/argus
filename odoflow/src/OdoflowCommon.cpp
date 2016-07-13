#include "odoflow/OdoflowCommon.h"

using namespace image_geometry;

namespace argus
{
	
bool UndistortPoints( const InterestPoints& points, 
                      const PinholeCameraModel& model,
                      bool undistort, bool normalize,
                      InterestPoints& undistorted )
{
	undistorted.clear();
	if( points.empty() ) { return true; }
	
	// TODO Check more
	if( std::isnan( model.fx() ) ) { return false; }
	
	// Perform undistortion
	cv::Mat distortionCoeffs;
	cv::Matx33d outputCameraMatrix;
	
	if( undistort ) { distortionCoeffs = model.distortionCoeffs(); }
	outputCameraMatrix = normalize ? cv::Matx33d::eye() : model.intrinsicMatrix();
	
	cv::undistortPoints( points, undistorted, model.intrinsicMatrix(),
	                     distortionCoeffs, cv::noArray(), outputCameraMatrix );
	return true;
}

void TransformPoints( const InterestPoints& points,
                      const PoseSE2& trans,
                      InterestPoints& transformed )
{
	if( points.size() == 0 ) { return; }
	Eigen::Matrix<double, 2, Eigen::Dynamic> pts( 2, points.size() );
	for( unsigned int i = 0; i < points.size(); ++i )
	{
		pts(0,i) = points[i].x;
		pts(1,i) = points[i].y;
	}

	pts = trans.ToTransform() * pts;
	transformed.reserve( points.size() );
	for( unsigned int i = 0; i < points.size(); ++i )
	{
		transformed.emplace_back( pts(0,i), pts(1,i) );
	}
}
	
}
