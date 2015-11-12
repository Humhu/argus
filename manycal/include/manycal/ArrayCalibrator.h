#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "manycal/slam3d.h"
#include "manycal/sclam_fiducial.h"
#include "manycal/OdometryGraph.hpp"

#include <isam/sclam_monocular.h>

#include "argus_msgs/ImageFiducialDetections.h"

#include "argus_utils/PoseSE3.h"

#include "fiducial_array/FiducialInfoManager.h"

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
	
	ros::Subscriber detectionSub;
	ros::Subscriber relPoseSub;
	
	fiducial_array::FiducialInfoManager fidManager;
	extrinsics_array::ExtrinsicsInfoManager camManager;
	
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
	
	/*! \brief Map from frame names to registrations. */
	std::unordered_map <std::string, FrameRegistration> frameRegistry;
	std::unordered_map <std::string, CameraRegistration> cameraRegistry;
	std::unordered_map <std::string, FiducialRegistration> fiducialRegistry;
	
	void RelPoseCallback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg );
	void DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg );
	
	void InitializeFrame( const std::string& frameName, ros::Time t );
	void InitializeCamera( const std::string& cameraName, ros::Time t );
	void InitializeFiducial( const std::string& fidName, ros::Time t );
	
	void CreateObservationFactor( CameraRegistration& camera,
	                              FrameRegistration& cameraFrame,
	                              FiducialRegistration& fiducial,
	                              FrameRegistration& fiducialFrame,
	                              const argus_msgs::FiducialDetection& detection,
								  ros::Time t );
	
};
	
} // end namespace manycal
