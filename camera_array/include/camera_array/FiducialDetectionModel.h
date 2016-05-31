#pragma once

#include "camera_array/SystemStates.h"

#include "lookup/LookupInterface.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"
#include "fiducials/FiducialInfoManager.h"
#include "fiducials/FiducialCommon.h"
#include "fieldtrack/TargetInfoManager.h"

namespace camera_array
{
	
// TODO Add min point distance filter
/*! \brief Generates fiducial detections. */
class FiducialDetectionModel
{
public:
	
	typedef std::shared_ptr<FiducialDetectionModel> Ptr;
	typedef std::vector<argus_msgs::FiducialDetection> Detections;
	
	FiducialDetectionModel( lookup::LookupInterface& interface );
	
	/*! \brief Generates fiducial detections for a camera, target, and the pose 
	 * of the target in the camera reference frame (usually the robot). All
	 * parameters must be lookup-retrievable. */
	Detections GenerateDetections( const std::string& cameraName,
	                               const std::string& targetName,
	                               const argus::PoseSE3& targetToCameraRef );
	
private:
	
	mutable extrinsics_array::ExtrinsicsInfoManager extrinsicsManager;
	mutable fiducials::FiducialInfoManager fiducialManager;
	mutable fieldtrack::TargetInfoManager targetManager;

	double minPointSeparation;
	
	void CheckCamera( const std::string& cameraName );
	void CheckTarget( const std::string& targetName );
	
};
	
}
