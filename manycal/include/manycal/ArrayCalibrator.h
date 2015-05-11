#ifndef _MANYCAL_ARRAY_CALIBRATOR_H_
#define _MANYCAL_ARRAY_CALIBRATOR_H_

#include <ros/ros.h>

#include <isam/isam.h>
#include <isam/slam_monocular.h>

#include <unordered_map>

namespace manycal 
{
	
	class ArrayCalibrator
	{
	public:
		
		ArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
		
	private:
		
		ros::NodeHandle nodeHandle;
		ros::NodeHandle privHandle;
		
		struct CameraRegistration
		{
			std::string name;
			isam::MonocularCamera cameraModel;
		};
		
		isam::Slam slam;
		std::unordered_map< std::string, CameraRegistration> registry;
		
	};
	
}

#endif
