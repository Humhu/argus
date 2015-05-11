#ifndef _V4L2D_CAMERA_MANAGER_H_
#define _V4L2D_CAMERA_MANAGER_H_

#include <ros/ros.h>

#include "argus_common/WorkerPool.hpp"

#include "v4l2_cam/CycleArray.h"
#include "v4l2_cam/EnableArrayCamera.h"
#include "v4l2_cam/DisableArrayCamera.h"
#include "v4l2_cam/DisableArray.h"

#include "v4l2_cam/SetStreaming.h"

#include <unordered_map>

namespace v4l2_cam
{

	/*! \brief Stateful camera connection manager and interface. */
	class CameraManager
	{
	public:
		
		typedef std::shared_ptr<CameraManager> Ptr;
		
		const std::string cameraName;
		
		CameraManager( const ros::NodeHandle& nh, const std::string& _cameraName );
		
		void ValidateConnection(); // TODO Return success?
		void EnableCamera();
		void DisableCamera();
		
	private:
		
		ros::NodeHandle nodeHandle;
		ros::ServiceClient setStreamingClient;
		
	};
	
	// TODO Unified management of camera extrinsics and intrinsics
	/*! \brief Represents a cluster of cameras. Presents methods to interact 
	 * with them as a networked array instead of individuals. */
	class CameraArray
	{
	public:
		
		CameraArray( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
		~CameraArray();
		
	private:
		
// 		typedef boost::unique_lock< boost::shared_mutex > WriteLock;
// 		typedef boost::shared_lock< boost::shared_mutex > ReadLock;
// 		boost::shared_mutex mutex;

		ros::NodeHandle nodeHandle;
		ros::NodeHandle privHandle;
		
		ros::ServiceServer cycleCamerasServer;
		ros::ServiceServer enableCameraServer;
		ros::ServiceServer disableCameraServer;
		ros::ServiceServer disableAllServer;
		
		argus_common::WorkerPool dispatchers;
		
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
		
		void EnableCamera( CameraManager::Ptr manager );
		void DisableCamera( CameraManager::Ptr manager );
	};
	
}

#endif
