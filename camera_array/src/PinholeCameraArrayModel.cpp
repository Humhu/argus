#include "camera_array/PinholeCameraArrayModel.h"

using namespace argus_utils;
using namespace camplex;

namespace camera_array
{

PinholeCameraArrayModel::PinholeCameraArrayModel()
{}

void PinholeCameraArrayModel::FromInfo( const CameraArrayInfo& info )
{
	
	cameraNames = info.cameraNames;
	for( unsigned int i = 0; i < cameraNames.size(); i++ )
	{
		PoseSE3 extrinsic = MsgToPose( info.extrinsics[i] );
		CameraCalibration cc( cameraNames[i], info.intrinsics[i] );
		CameraRegistration reg = { extrinsic.Inverse(), cc };
		registry.push_back( reg );
	}
}

std::vector< cv::Rect > PinholeCameraArrayModel::GetRois() const
{
	std::vector< cv::Rect > rois( registry.size() );
	for( unsigned int i = 0; i < registry.size(); i++ )
	{
		rois[i] = registry[i].intrinsic.GetRoi();
	}
	return rois;
}
	
std::vector<CameraObservation> PinholeCameraArrayModel::Project( const Eigen::Vector3d& point )
{
	// Convert from x-forward to z-forward
	std::vector<CameraObservation> observations( registry.size() );
	for( unsigned int i = 0; i < registry.size(); i++ )
	{
		Eigen::Vector3d relPoint = registry[i].extrinsicInverse.ToTransform()*point;
		if( relPoint(0) <= 0 ) 
		{
			observations[i].valid = false;
		}
		else
		{
			observations[i].valid = true;
			Eigen::Vector3d camPoint( -point(1), -point(2), point(0) );
			observations[i].imageCoordinates = ProjectPoint( registry[i].intrinsic, camPoint );
		}
	}
	return observations;
}

Eigen::Vector2d PinholeCameraArrayModel::ProjectPoint( const CameraCalibration& intrinsic,
													   const Eigen::Vector3d& point )
{
	double u = point(0)*intrinsic.GetFx()/point(2) + intrinsic.GetCx();
	double v = point(1)*intrinsic.GetFy()/point(2) + intrinsic.GetCy();
	return Eigen::Vector2d( u, v );
}

} // end namespace camera_array
