#include "v4l2_cam/CameraManager.h"
#include "v4l2_cam/SetStreaming.h"
#include "v4l2_cam/CycleCameras.h"

#include <boost/foreach.hpp>

using namespace argus_common;

namespace v4l2_cam
{
	
	CameraManager::CameraManager( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: nodeHandle( nh ), privHandle( ph ),
		cycleCamerasServer( privHandle.advertiseService(
							"cycle_cameras",
							&CameraManager::CycleCamerasService,
							this ) )
	{
		
		std::vector<std::string> cameraNames;
		privHandle.getParam( "cameras", cameraNames );
		
		BOOST_FOREACH( const std::string& name, cameraNames )
		{
			RegisterCamera( name );
		}
		
		dispatchers.SetNumWorkers( cameraNames.size() );
		dispatchers.StartWorkers();
	}
	
	CameraManager::~CameraManager()
	{
		dispatchers.StopWorkers();
	}
	
	void CameraManager::RegisterCamera( const std::string& cameraName )
	{
		CameraRegistration registration;
		registration.name = cameraName;
		registration.setStreamingClient = 
			nodeHandle.serviceClient<SetStreaming>( cameraName + "/set_streaming", true );
		registry.push_back( registration );
	}
	
	void CameraManager::ServiceJob( CameraRegistration reg, SetStreaming srv )
	{
		std::cout << "Calling job..." << std::endl;
		bool result = reg.setStreamingClient.call( srv );
		std::cout << "Job finished with status: " << result << std::endl;
	}
	
	bool CameraManager::CycleCamerasService( v4l2_cam::CycleCameras::Request& req,
								  v4l2_cam::CycleCameras::Response& res )
	{

		BOOST_FOREACH( CameraRegistration& registration, registry )
		{
			if( !registration.setStreamingClient.isValid() )
			{
				ROS_WARN_STREAM( "Lost connection to camera " << registration.name
					<< ". Attempting to reconnect..." );
				registration.setStreamingClient = 
					nodeHandle.serviceClient<SetStreaming>( registration.name + "/set_streaming", true );
			}
		}
		
		SetStreaming srv;
		BOOST_FOREACH( const CameraRegistration& registration, registry )
		{
			srv.request.enableStreaming = true;
			srv.request.numFramesToStream = req.numToCapture;
			WorkerPool::Job job = 
				boost::bind( &CameraManager::ServiceJob, registration, srv );
			dispatchers.EnqueueJob( job );
		}
		
		return true;
	}
	
}
