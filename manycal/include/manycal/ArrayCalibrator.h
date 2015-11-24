#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "manycal/sclam_fiducial.h"
#include <isam/sclam_monocular.h>
#include "manycal/OdometryGraph.hpp"

#include "lookup/LookupInterface.h"
#include "argus_msgs/ImageFiducialDetections.h"
#include "argus_utils/PoseSE3.h"
#include "fiducials/FiducialInfoManager.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

#include <unordered_map>

namespace manycal 
{
	
/*! \brief Calibrates arrays of cameras and fiducials. */
class ArrayCalibrator
{
public:
	
	ArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	~ArrayCalibrator();
	
	void WriteResults();
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	lookup::LookupInterface lookup;
	fiducials::FiducialInfoManager fiducialManager;
	extrinsics_array::ExtrinsicsInfoManager extrinsicsManager;
	
	int targetNumObservations;
	
	/*! \brief Stores subscriptions to odometry and detection inputs. */
	struct TargetRegistration
	{
		ros::Subscriber detectionSub;
		ros::Subscriber odometrySub;
	};
	std::unordered_map <std::string, TargetRegistration> targetRegistry;
	
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
	
	typedef OdometryGraph<isam::PoseSE3, ros::Time> OdometryGraphSE3;
	struct FrameRegistration
	{
		bool initialized;
		OdometryGraphSE3::Ptr poses;
		bool optimizePoses;
	};
	
	isam::Slam::Ptr slam;
	std::vector<isam::FiducialFactor::Ptr> observations;
	
	/*! \brief Map from frame names to registrations. */
	typedef std::unordered_map <std::string, FrameRegistration> FrameRegistry;
	FrameRegistry frameRegistry;
	typedef std::unordered_map <std::string, CameraRegistration> CameraRegistry;
	CameraRegistry cameraRegistry;
	typedef std::unordered_map <std::string, FiducialRegistration> FiducialRegistry;
	FiducialRegistry fiducialRegistry;
	
	void OdometryCallback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg );
	void DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg );
	
	void CreateFrame( const std::string& frameName, const ros::Time& t );
	
	/*! \brief Initialize a new camera. Uses the camera manager to determine parent 
	 * reference. Throws an error if camera cannot be found. */
	void InitializeCamera( const std::string& cameraName );
	
	/*! \brief Initialize a new fiducial. Uses the extrinsics manager to determine 
	 * parent reference. Returns success. */
	void InitializeFiducial( const std::string& fidName );
	
	void CreateObservationFactor( CameraRegistration& camera,
	                              FrameRegistration& cameraFrame,
	                              FiducialRegistration& fiducial,
	                              FrameRegistration& fiducialFrame,
	                              const argus_msgs::FiducialDetection& detection,
								  ros::Time t );
	
};
	
} // end namespace manycal
