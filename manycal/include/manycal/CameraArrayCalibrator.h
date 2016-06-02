#pragma once

#include <ros/ros.h>
#include <isam/sclam_monocular.h>
#include <isam/Slam.h>

#include "lookup/LookupInterface.h"
#include "fiducials/FiducialInfoManager.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

#include "manycal/ManycalCommon.h"
#include "manycal/WriteCalibration.h"
#include "manycal/sclam_fiducial.h"
#include "argus_msgs/ImageFiducialDetections.h"

namespace argus
{

/*! \brief Calibrates an array of synchronized cameras. Assumes fiducials can move
 * around arbitrarily. */
class CameraArrayCalibrator
{
public:
	
	CameraArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	
	bool WriteResults( manycal::WriteCalibration::Request& req, 
	                   manycal::WriteCalibration::Response& res );
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	ros::Subscriber detSub;
	ros::ServiceServer writeServer;

	LookupInterface lookupInterface;
	FiducialInfoManager fiducialManager;
	ExtrinsicsInfoManager extrinsicsManager;
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
	
	void RegisterCamera( const std::string& name, const argus::PoseSE3& pose,
						 bool addPrior = false );
	void RegisterFiducial( const std::string& name );
	
};
	
}
