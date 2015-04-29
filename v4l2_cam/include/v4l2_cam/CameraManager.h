#ifndef _V4L2D_CAMERA_MANAGER_H_
#define _V4L2D_CAMERA_MANAGER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "v4l2_cam/WorkerPool.h"
#include "v4l2_cam/CycleCameras.h"
#include "v4l2_cam/SetStreaming.h"

namespace v4l2_cam
{
	
	/*! \brief Represents a cluster of cameras. Presents methods to interact 
	 * with them as a network. */
	class CameraManager
	{
	public:
		
		CameraManager( ros::NodeHandle& nh, ros::NodeHandle& ph );
		~CameraManager();
		
	private:
		
		typedef boost::unique_lock< boost::shared_mutex > Lock;
		boost::shared_mutex mutex;

		ros::NodeHandle nodeHandle;
		ros::NodeHandle privHandle;
		ros::ServiceServer cycleCamerasServer;
		
		WorkerPool dispatchers;
		
		boost::condition_variable_any commandBlock;
		
		struct CameraRegistration
		{
			std::string name;
			ros::ServiceClient setStreamingClient;
		};
		
		std::vector< CameraRegistration > registry;
		
		void RegisterCamera( const std::string& cameraName );
		static void ServiceJob( CameraRegistration reg, SetStreaming srv );
		bool CycleCamerasService( v4l2_cam::CycleCameras::Request& req,
								  v4l2_cam::CycleCameras::Response& res );
		
	};
	
}

#endif
