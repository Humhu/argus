#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <tf/transform_broadcaster.h>

#include <isam/slam_tag.h>
#include <isam/SlamInterface.h>

#include <unordered_map>

#include "manycal/PoseGraph.hpp"

#include "argus_msgs/TagDetection.h"
#include "argus_utils/PoseSE3.h"

namespace manycal 
{
/*! \brief Calibrates an array of cameras by using fiducial detections. 
 * Reads from the output of ArrayCapturer. */
class ArrayCalibrator
{
public:
	
	ArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	~ArrayCalibrator();
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	ros::Subscriber detectionSub;
	
	tf::TransformBroadcaster transformPub;
	
	struct CameraRegistration
	{
		isam::MonocularIntrinsics_Node::Ptr intrinsics;
		isam::Pose3d_Node::Ptr extrinsics;
		isam::Pose3d_Factor::Ptr prior;
	};
	
	struct TagRegistration
	{
		std::string name;
		unsigned int id;
		std::string family;
		isam::TagIntrinsics_Node::Ptr intrinsics;
		bool initialized; // Initialized in pose graph
	};
	
	isam::SlamInterface slam;
	PoseGraph3d poseGraph;
	YAML::Node cameraConfigLog;
	YAML::Node tagConfigLog;
	unsigned int detectionCounter;
	
	typedef std::unordered_map< std::string, CameraRegistration > CameraRegistry;
	typedef std::unordered_map< std::string, TagRegistration > TagRegistry;
	
	/*! \brief Mapping from camera names to registrations. */
	CameraRegistry cameraRegistry;
	/*! \brief Mapping from tag hashes to registrations. */
	TagRegistry tagRegistry;
	
	void DetectionCallback( const argus_msgs::TagDetection::ConstPtr& msg );
	
	void ParseCamera( const std::string& name, YAML::Node cam );
	void ParseTag( const std::string& name, YAML::Node tag );
	
	void InitializeTag( const std::string& key, const argus_msgs::TagDetection& msg );
	
	// Turns a tag detection into a string hash
	static std::string GetTagKey( const std::string& tagFamily, unsigned int id );

	// Optimize the graph and publish all new poses
	void UpdateCalibration();
	void PublishPose( const std::string& name, const isam::Pose3d& pose, ros::Time time );
	
	void InitializeCamera( const std::string& name );
	void InitializeTag( const std::string& camName, const argus_msgs::TagDetection& msg,
						double size );
	
	static std::string GetTagName( unsigned int id );
	
};
	
} // end namespace manycal
