#ifndef _MANYCAL_ARRAY_CALIBRATOR_H_
#define _MANYCAL_ARRAY_CALIBRATOR_H_

#include <ros/ros.h>

#include <isam/slam_tag.h>
#include <isam/SlamInterface.h>

#include <unordered_map>

#include "manycal/PoseGraph.hpp"
#include "argus_msgs/TagDetection.h"
#include "argus_common/ArgusTypes.h"

namespace manycal 
{
	/*! \brief Calibrates an array of cameras by using fiducial detections. */
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
			isam::TagIntrinsics_Node::Ptr intrinsics;
			isam::Pose3d_Node::Ptr extrinsics;
		};
		
		isam::SlamInterface slam;
		PoseGraph3d poseGraph;
		
		typedef std::unordered_map< std::string, CameraRegistration > CameraRegistry;
		typedef std::unordered_map< unsigned int, TagRegistration > TagRegistry;
		CameraRegistry cameraRegistry;
		TagRegistry tagRegistry;
		double tagSize; // HACK
		
		void DetectionCallback( const argus_msgs::TagDetection::ConstPtr& msg );
		
		void InitializeCamera( const std::string& name );
		void InitializeTag( const std::string& camName, const argus_msgs::TagDetection& msg,
							double size );
		
	};
	
}

#endif
