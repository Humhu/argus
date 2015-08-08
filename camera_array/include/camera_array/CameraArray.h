#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include "argus_utils/WorkerPool.hpp"
#include "argus_utils/PoseSE3.h"

#include "camplex/CameraCalibration.h"
#include "camplex/SetStreaming.h"
#include "camplex/CaptureFrames.h"

#include "camera_array/CameraArrayInfoManager.h"
#include "camera_array/GetCameraArrayInfo.h"

#include <yaml-cpp/yaml.h>

#include <unordered_map>

#include <tf/transform_broadcaster.h>

namespace camera_array
{
	/*! \brief Represents a cluster of cameras. Presents methods to interact 
	 * with them as a networked array instead of individuals. 
	 * TODO Maintains a set of tf frames for the extrinsics?
	 * 
	 * Calibration YAML format:
	 * array_name: [name of the array]
	 * cameras:
	 *   [camera_0_name]:
	 *     extrinsics:
	 *       position: [pose of camera_0 relative to array reference frame]
	 *       quaternion: [quat]
	 *   [camera_1_name]:
	 *     etc.
	 */
	class CameraArray
	{
	public:
		
		typedef std::shared_ptr<CameraArray> Ptr;
		
		CameraArray( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
		~CameraArray();
		
		bool CallSetStreaming( const std::string& cameraName, bool stream );
		bool CallCaptureFrames( const std::string& cameraName, unsigned int num );
		
	private:
		
		ros::NodeHandle nodeHandle;
		ros::NodeHandle privHandle;

		CameraArrayInfoManager extrinsicsManager;

		// Service handlers
		ros::ServiceServer setStreamingServer;
		ros::ServiceServer captureFramesServer;
		
// 		argus_utils::WorkerPool dispatchers;
// 		boost::condition_variable_any commandBlock;
		
		/*! \brief Status and registration entry for each camera. */
		
		struct CameraRegistration
		{
			ros::ServiceClient setStreamingClient;
			ros::ServiceClient captureFramesClient;
			
			CameraRegistration( ros::NodeHandle& nh, const std::string& cameraName );
			
			bool CallSetStreaming( bool stream );
			bool CallCaptureFrames( unsigned int num );
			bool CallGetCameraInfo( sensor_msgs::CameraInfo& info );
		};
		
		typedef std::unordered_map< std::string, CameraRegistration > CameraRegistry;
		CameraRegistry registry;
		
		bool SetStreamService( camplex::SetStreaming::Request& req,
							   camplex::SetStreaming::Response& res );
		bool CaptureFramesService( camplex::CaptureFrames::Request& req,
								   camplex::CaptureFrames::Response& res );
		
		/*! \brief Register a camera to the array using its node name. */
		void RegisterCamera( const std::string& cameraName );
	};
	
}
