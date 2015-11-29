#pragma once

#include <ros/ros.h>
#include <isam/sclam_monocular.h>
#include <isam/Slam.h>

#include "lookup/LookupInterface.h"
#include "fiducials/FiducialInfoManager.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

#include "manycal/ManycalCommon.h"
#include "manycal/sclam_fiducial.h"
#include "argus_msgs/ImageFiducialDetections.h"

namespace manycal
{

/*! \brief Calibrates an array of synchronized cameras. Assumes independent
 * fiducial detections. */
class CameraArrayCalibrator
{
public:
	
	CameraArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	
	void WriteResults();
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	ros::Subscriber detSub;
	
	lookup::LookupInterface lookupInterface;
	fiducials::FiducialInfoManager fiducialManager;
	extrinsics_array::ExtrinsicsInfoManager extrinsicsManager;
	std::string referenceFrame;
	
	struct CameraRegistration
	{
		isam::PoseSE3_Node::Ptr extrinsics;
		isam::MonocularIntrinsics_Node::Ptr intrinsics;
		isam::PoseSE3_Prior::Ptr extrinsicsPrior;
	};
	
	struct FiducialRegistration
	{
		isam::FiducialIntrinsics_Node::Ptr intrinsics;
		std::map <ros::Time, isam::PoseSE3_Node::Ptr> poses;
	};
	
	isam::Slam::Ptr slam;
	std::vector <isam::FiducialFactor::Ptr> observations;
	std::vector <argus_msgs::ImageFiducialDetections::ConstPtr> cachedObservations;
	
	typedef std::unordered_map <std::string, CameraRegistration> CameraRegistry;
	CameraRegistry cameraRegistry;
	
	typedef std::unordered_map <std::string, FiducialRegistration> FiducialRegistry;
	FiducialRegistry fiducialRegistry;
	
	void DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg );
	bool ProcessDetection( const argus_msgs::ImageFiducialDetections::ConstPtr& msg );
	bool InitializeCamera( const argus_msgs::ImageFiducialDetections::ConstPtr& msg );
	void ProcessCache();
	
	bool LoadFiducialIntrinsics( const std::string& name );
	bool LoadExtrinsicsPrior( const std::string& name );
	void RegisterCamera( const std::string& name, const argus_utils::PoseSE3& pose,
						 bool addPrior = false );
	void RegisterFiducial( const std::string& name );
	
};
	
}
