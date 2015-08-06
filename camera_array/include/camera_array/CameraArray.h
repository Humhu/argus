#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include "argus_utils/WorkerPool.hpp"

#include "camera_array/CycleArray.h"
#include "camera_array/EnableArrayCamera.h"
#include "camera_array/DisableArrayCamera.h"
#include "camera_array/DisableArray.h"
#include "camera_array/ListArrayCameras.h"

#include "camplex/SetStreaming.h"

#include <unordered_map>

namespace camera_array
{

	/*! \brief Stateful camera connection manager and interface. */
	class CameraManager
	{
	public:
		
		typedef std::shared_ptr<CameraManager> Ptr;
		
		const std::string cameraName;
		
		CameraManager( const ros::NodeHandle& nh,
					   image_transport::CameraPublisher& ipub,
					   const std::string& _cameraName );
		~CameraManager();
		
		void ValidateConnection(); // TODO Return success?
		void EnableCamera();
		void DisableCamera();
		
	private:
		
		ros::NodeHandle nodeHandle;
		ros::NodeHandle privHandle;
		image_transport::ImageTransport imagePort;
		image_transport::CameraSubscriber imageSub;
		image_transport::CameraPublisher& imagePub;
		ros::ServiceClient setStreamingClient;
		
		void ImageCallback( const sensor_msgs::Image::ConstPtr& msg,
						    const sensor_msgs::CameraInfo::ConstPtr& info_msg );
		
	};
	
	// TODO Unified management of camera extrinsics and intrinsics
	/*! \brief Represents a cluster of cameras. Presents methods to interact 
	 * with them as a networked array instead of individuals. */
	// NOTE All the methods are unsychronized because they are expected to be 
	// called by a single ROS spinner thread
	class CameraArray
	{
	public:
		
		typedef std::shared_ptr<CameraArray> Ptr;
		
		CameraArray( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
		~CameraArray();
		
	private:
		
		ros::NodeHandle nodeHandle;
		ros::NodeHandle privHandle;
		
		// Service handlers
		ros::ServiceServer cycleArrayServer;
		ros::ServiceServer enableCameraServer;
		ros::ServiceServer disableCameraServer;
		ros::ServiceServer disableAllServer;
		ros::ServiceServer listCamerasServer;
		
		image_transport::ImageTransport imagePort;
		image_transport::CameraPublisher imagePub;
		
		argus_utils::WorkerPool dispatchers;
		
		boost::condition_variable_any commandBlock;
		
		/*! \brief Status and registration entry for each camera. */
		
		typedef std::unordered_map< std::string, CameraManager::Ptr > CameraRegistry;
		CameraRegistry registry;
		
		bool CycleArrayService( CycleArray::Request& req,
								CycleArray::Response& res );
		bool EnableCameraService( EnableArrayCamera::Request& req,
								  EnableArrayCamera::Response& res );
		bool DisableCameraService( DisableArrayCamera::Request& req,
								   DisableArrayCamera::Response& res );
		bool DisableArrayService( DisableArray::Request& req,
								  DisableArray::Response& res );
		bool ListCamerasService( ListArrayCameras::Request& req,
								 ListArrayCameras::Response& res );
		
		void EnableCamera( CameraManager::Ptr manager );
		void DisableCamera( CameraManager::Ptr manager );
	};
	
}
