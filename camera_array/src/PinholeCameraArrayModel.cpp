#include "camera_array/PinholeCameraArrayModel.h"

#include <boost/foreach.hpp>
#include <assert.h>

using namespace argus_utils;

namespace camera_array
{

PinholeCameraArrayModel::PinholeCameraArrayModel()
{}

void PinholeCameraArrayModel::FromInfo( const CameraArrayInfo& info )
{
	assert( extrinsics.cameraNames.size() == info.size() );
	
	cameraNames = info.cameraNames;
	for( unsigned int i = 0; i < cameraNames.size(); i++ )
	{
		CameraRegistration reg;
		reg.extrinsic = MsgToPose( info.extrinsics[i] );
		reg.cameraModel.fromCameraInfo( info.info[i] );
		registry.push_back( reg );
	}
}

PinholeCameraArrayModel::Points PinholeCameraArrayModel::Project( const cv::Point3d& point )
{
	Points points;
	BOOST_FOREACH( const CameraRegistration& reg, registry )
	{
		Eigen::Vector3d x( point.x, point.y, point.z );
		Eigen::Vector3d relX = reg.extrinsic.Inverse().ToTransform()*x;
		cv::Point3d relPoint( relX(0), relX(1), relX(2) );
		points.push_back( reg.cameraModel.project3dToPixel( relPoint ) );
	}
	return points;
}



} // end namespace camera_array
