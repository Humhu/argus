#include "odoflow/OdoflowCommon.h"

using namespace image_geometry;

namespace odoflow
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
	
}
