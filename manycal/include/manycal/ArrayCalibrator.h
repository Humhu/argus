#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "manycal/slamse3.h"
#include <isam/sclam_monocular.h>

#include "manycal/sclam_fiducial.h"
#include "manycal/PoseGraph.hpp"
#include "manycal/OdometryGraph.hpp"
#include "manycal/StaticPoseGraph.hpp"
#include "manycal/JumpPoseGraph.hpp"
#include "manycal/WriteCalibration.h"

#include "fieldtrack/TargetInfoManager.h"

#include "lookup/LookupInterface.h"
#include "argus_msgs/ImageFiducialDetections.h"
#include "argus_utils/geometry/PoseSE3.h"
#include "fiducials/FiducialInfoManager.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

#include <unordered_map>

namespace argus 
{

/*! \brief Calibrates arrays of cameras and fiducials. */
class ArrayCalibrator
{
public:
	
	typedef PoseGraph<isam::PoseSE3, ros::Time> PoseGraphType;
	typedef StaticPoseGraph<isam::PoseSE3, ros::Time> StaticPoseGraphType;
	typedef OdometryGraph<isam::PoseSE3, ros::Time> OdometryGraphType;
	typedef JumpPoseGraph<isam::PoseSE3, ros::Time> JumpPoseGraphType;

	ArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	
	bool WriteResults( manycal::WriteCalibration::Request& req, 
	                   manycal::WriteCalibration::Response& res );
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	ros::ServiceServer writeServer;

	LookupInterface lookup;
	FiducialInfoManager fiducialManager;
	ExtrinsicsInfoManager extrinsicsManager;
	TargetInfoManager targetManager;

	/*! \brief Stores subscriptions to odometry and detection inputs. */
	struct TargetRegistration
	{
		ros::Subscriber detectionSub;
		ros::Subscriber odometrySub;

		PoseGraphType::Ptr poses;
		bool initialized;
		bool optimizePoses;
		bool optimizeCameras;
		bool optimizeFiducials;
	};
	typedef std::unordered_map <std::string, TargetRegistration> TargetRegistry;
	TargetRegistry targetRegistry;
	
	struct CameraRegistration
	{
		std::string name;
		std::shared_ptr <isam::PoseSE3_Node> extrinsics;
		std::shared_ptr <isam::MonocularIntrinsics_Node> intrinsics;
		std::shared_ptr <isam::PoseSE3_Prior> extrinsicsPrior;
		std::shared_ptr <isam::MonocularIntrinsics_Prior> intrinsicsPrior;
		bool optimizeExtrinsics;
		bool optimizeIntrinsics;
	};
	
	struct FiducialRegistration
	{
		std::string name;
		std::shared_ptr <isam::PoseSE3_Node> extrinsics;
		std::shared_ptr <isam::FiducialIntrinsics_Node> intrinsics;
		std::shared_ptr <isam::PoseSE3_Prior> extrinsicsPrior;
		std::shared_ptr <isam::FiducialIntrinsics_Prior> intrinsicsPrior;
		bool optimizeExtrinsics;
		bool optimizeIntrinsics;
	};
	
	isam::Slam::Ptr slam;
	std::vector<isam::FiducialFactor::Ptr> observations;
	
	/*! \brief Map from frame names to registrations. */
	typedef std::unordered_map <std::string, CameraRegistration> CameraRegistry;
	CameraRegistry cameraRegistry;
	typedef std::unordered_map <std::string, FiducialRegistration> FiducialRegistry;
	FiducialRegistry fiducialRegistry;
	
	void OdometryCallback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg );
	void DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg );
	
	void RegisterTarget( const std::string& targetName, const YAML::Node& yaml );

	/*! \brief Initialize a new camera. Uses the camera manager to determine parent 
	 * reference. Returns success. */
	void RegisterCamera( const std::string& cameraName, bool calibrate );
	
	/*! \brief Initialize a new fiducial. Uses the extrinsics manager to determine 
	 * parent reference. Returns success. */
	void RegisterFiducial( const std::string& fidName, bool calibrate );

	
	void CreateObservationFactor( CameraRegistration& camera,
	                              TargetRegistration& cameraFrame,
	                              FiducialRegistration& fiducial,
	                              TargetRegistration& fiducialFrame,
	                              const argus_msgs::FiducialDetection& detection,
								  ros::Time t );
	
};
	
} // end namespace manycal
