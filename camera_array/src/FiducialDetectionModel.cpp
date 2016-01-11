#include "camera_array/FiducialDetectionModel.h"
#include "camplex/CameraCalibration.h"
#include <boost/foreach.hpp>

using namespace argus_msgs;
using namespace argus_utils;
using namespace fieldtrack;

namespace camera_array
{
	
FiducialDetectionModel::FiducialDetectionModel( lookup::LookupInterface& interface )
: extrinsicsManager( interface ), fiducialManager( interface ), 
targetManager( interface )
{
  // TODO
  minPointSeparation = 10.0;
}

FiducialDetectionModel::Detections
FiducialDetectionModel::GenerateDetections( const std::string& cameraName,
                                            const std::string& targetName,
                                            const PoseSE3& targetToCameraRef )
{
	CheckCamera( cameraName );
	CheckTarget( targetName );
	
	std::vector<FiducialDetection> detections;
	
	const PoseSE3& cameraExtrinsics = extrinsicsManager.GetInfo( cameraName ).extrinsics;
	PoseSE3 targetToCam = cameraExtrinsics.Inverse() * targetToCameraRef;
	
	// TODO 
	camplex::CameraCalibration cameraModel( "fake", 550, 550, 320, 240, 640, 480 );
	
	const TargetInfo& info = targetManager.GetInfo( targetName );
	BOOST_FOREACH( const TargetInfo::FiducialGroupRegistry::value_type& group, info.fiducialGroups )
	{
		BOOST_FOREACH( const std::string& fidName, group.second.fiducials )
		{
			const PoseSE3& fidExtrinsics = extrinsicsManager.GetInfo( fidName ).extrinsics;
			const fiducials::Fiducial& fidIntrinsics = fiducialManager.GetInfo( fidName );
			PoseSE3 fidToCam = targetToCam * fidExtrinsics;
			FiducialDetection detection;
			detection.name = fidName;
			bool valid = fiducials::ProjectDetection( fidIntrinsics, 
	                                                  cameraModel,
	                                                  fidToCam,
			                                          detection );
			// 		ROS_INFO_STREAM( "Camera " << cameraName << " fiducial " << fidName 
					 // 			<< " rel pose " << fidToCam );
			// 		for( unsigned int i = 0; i < detection.points.size(); i++ )
			  // 		{
			  // 			ROS_INFO_STREAM( "Detected point: " << detection.points[i].x
						 //	<< ", " << detection.points[i].y );
				// 		}
			if( !valid ) { continue; }
			double minDist = fiducials::FindMinDistance( detection.points );
			if( minDist < minPointSeparation ) { continue; }
			//		ROS_INFO_STREAM( "Camera " << cameraName << " detects " << fidName );
			detections.push_back( detection );
		}
	}
	
	return detections;
}
	
void FiducialDetectionModel::CheckCamera( const std::string& cameraName )
{
	if( !extrinsicsManager.CheckMemberInfo( cameraName ) )
	{
		ROS_ERROR_STREAM( "Could not read extrinsics for camera: " << cameraName );
		exit( -1 );
	}
	// TODO Implement!
// 	if( cameraManager.HasMember( cameraName ) )
// 	{
// 		if( !cameraManager.ReadMemberInfo( cameraName ) )
// 		{
// 			ROS_ERROR_STREAM( "Could not read intrinsics for camera: " << cameraName );
// 			exit( -1 );
// 		}
// 	}
}

void FiducialDetectionModel::CheckTarget( const std::string& targetName )
{
	if( !targetManager.CheckMemberInfo( targetName ) )
	{
		ROS_ERROR_STREAM( "Could not read target info for: " << targetName );
		exit( -1 );
	}
	
	TargetInfo& info = targetManager.GetInfo( targetName );
	BOOST_FOREACH( const TargetInfo::FiducialGroupRegistry::value_type& group, 
	               info.fiducialGroups )
	{
		BOOST_FOREACH( const std::string& fiducialName, group.second.fiducials )
		{
			if( !fiducialManager.CheckMemberInfo( fiducialName ) )
			{
				ROS_ERROR_STREAM( "Could not read fiducial info for: " << fiducialName );
				exit( -1 );
			}
			if( !extrinsicsManager.CheckMemberInfo( fiducialName ) )
			{
				ROS_ERROR_STREAM( "Could not read fiducial extrinsics for: " << fiducialName );
				exit( -1 );
			}
		}
	}
}

}
