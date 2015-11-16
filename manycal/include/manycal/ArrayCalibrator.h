#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "manycal/slam3d.h"
#include "manycal/sclam_fiducial.h"
#include "manycal/OdometryGraph.hpp"

#include <isam/sclam_monocular.h>

#include "lookup/LookupInterface.h"
#include "argus_msgs/ImageFiducialDetections.h"
#include "argus_utils/PoseSE3.h"
#include "fiducials/FiducialInfoManager.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

#include <unordered_map>

namespace manycal 
{
	
template <>
double index_difference <ros::Time>( const ros::Time& a, const ros::Time& b )
{
	return (a - b).toSec();
}
	
/*! \brief Calibrates arrays of cameras and fiducials. */
class ArrayCalibrator
{
public:
	
	ArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	~ArrayCalibrator();
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	lookup::LookupInterface lookup;
	fiducials::FiducialInfoManager fiducialManager;
	extrinsics_array::ExtrinsicsInfoManager extrinsicsManager;
	
	/*! \brief Stores subscriptions to odometry and detection inputs. */
	struct TargetRegistration
	{
		ros::Subscriber detectionSub;
		ros::Subscriber odometrySub;
	};
	std::unordered_map <std::string, TargetRegistration> targetRegistry;
	
	struct CameraRegistration
	{
		std::shared_ptr <isam::PoseSE3_Node> extrinsics;
		std::shared_ptr <isam::MonocularIntrinsics_Node> intrinsics;
		bool optimizeExtrinsics;
		bool optimizeIntrinsics;
	};
	
	struct FiducialRegistration
	{
		std::shared_ptr <isam::PoseSE3_Node> extrinsics;
		std::shared_ptr <isam::FiducialIntrinsics_Node> intrinsics;
		bool optimizeExtrinsics;
		bool optimizeIntrinsics;
	};
	
	typedef OdometryGraph<isam::PoseSE3, ros::Time> OdometryGraphSE3;
	struct FrameRegistration
	{
		OdometryGraphSE3::Ptr poses;
		bool optimizePoses;
	};
	
	isam::Slam::Ptr slam;
	std::vector<isam::FiducialFactor::Ptr> observations;
	
	/*! \brief Map from frame names to registrations. */
	std::unordered_map <std::string, FrameRegistration> frameRegistry;
	std::unordered_map <std::string, CameraRegistration> cameraRegistry;
	std::unordered_map <std::string, FiducialRegistration> fiducialRegistry;
	
	void OdometryCallback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg );
	void DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg );
	
	/*! \brief Initialize a new reference frame. Returns success. */
	bool InitializeFrame( const std::string& frameName, ros::Time t );
	
	/*! \brief Initialize a new camera. Uses the camera manager to determine parent 
	 * reference. Returns success. */
	bool InitializeCamera( const std::string& cameraName, ros::Time t );
	
	/*! \brief Initialize a new fiducial. Uses the extrinsics manager to determine 
	 * parent reference. Returns success. */
	bool InitializeFiducial( const std::string& fidName, ros::Time t );
	
	void CreateObservationFactor( CameraRegistration& camera,
	                              FrameRegistration& cameraFrame,
	                              FiducialRegistration& fiducial,
	                              FrameRegistration& fiducialFrame,
	                              const argus_msgs::FiducialDetection& detection,
								  ros::Time t );
	
};
	
} // end namespace manycal
