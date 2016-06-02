#pragma once

#include <ros/ros.h>
#include <isam/sclam_monocular.h>

#include "lookup/LookupInterface.h"
#include "fiducials/FiducialInfoManager.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

#include "manycal/ManycalCommon.h"
#include "manycal/sclam_fiducial.h"
#include "argus_msgs/ImageFiducialDetections.h"

namespace argus
{

// TODO Add service calls to write results
/*! \brief Uses a single camera to estimate the extrinsics for an array of fiducials. 
 * Fiducials are initialized as they are observed by the camera, with the first
 * fiducial placed at the origin. Consecutive observations are initialized by using
 * the last estimated pose of the camera. */
class FiducialArrayCalibrator
{
public:
	
	FiducialArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	~FiducialArrayCalibrator();
	
	void WriteResults();
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	ros::Subscriber detSub;

	LookupInterface lookupInterface;
	FiducialInfoManager fiducialManager;
	ExtrinsicsInfoManager extrinsicsManager;

	std::string sourceCamera;
	std::string referenceFrame;
	isam::MonocularIntrinsics_Node::Ptr cameraIntrinsics;
	
	struct FiducialRegistration
	{
		isam::PoseSE3_Node::Ptr extrinsics;
		isam::FiducialIntrinsics_Node::Ptr intrinsics;
		isam::PoseSE3_Prior::Ptr extrinsicsPrior;
	};
	
	isam::Slam::Ptr slam;
	isam::PoseSE3_Node::Ptr fiducialReference;
	std::vector <isam::PoseSE3_Node::Ptr> cameraPoses;
	std::vector <isam::FiducialFactor::Ptr> observations;
	
	typedef std::unordered_map <std::string, FiducialRegistration> FiducialRegistry;
	FiducialRegistry fiducialRegistry;
	
	void DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg );
	
	/*! \brief Checks if the specified fiducial has a pose guess. */
	bool HasExtrinsicsPrior( const std::string& name );
	bool HasIntrinsicsPrior( const std::string& name );
	
	/*! \brief Creates the fiducial and reads its intrinsics via lookup. */
	bool RegisterFiducial( const std::string& name, const argus::PoseSE3& pose,
	                       bool addPrior = false );
	
};
	
}
