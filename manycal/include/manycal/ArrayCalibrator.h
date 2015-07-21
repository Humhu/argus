#ifndef _MANYCAL_ARRAY_CALIBRATOR_H_
#define _MANYCAL_ARRAY_CALIBRATOR_H_

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <isam/slam_tag.h>
#include <isam/SlamInterface.h>

#include <unordered_map>

#include "manycal/PoseGraph.hpp"

#include "argus_msgs/TagDetection.h"
#include "argus_common/PoseSE3.h"

namespace manycal 
{
	/*! \brief Calibrates an array of cameras by using fiducial detections. Initializes from a 
	 * log file. */
	class ArrayCalibrator
	{
	public:
		
		ArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
		~ArrayCalibrator();
		
	private:
		
		ros::NodeHandle nodeHandle;
		ros::NodeHandle privHandle;
		
		ros::Subscriber detectionSub;
		
		struct CameraRegistration
		{
			isam::MonocularIntrinsics_Node::Ptr intrinsics;
			isam::Pose3d_Node::Ptr extrinsics;
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

	};
	
}

#endif
