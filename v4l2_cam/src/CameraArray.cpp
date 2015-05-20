#include "v4l2_cam/CameraArray.h"
#include "v4l2_cam/SetStreaming.h"
#include "v4l2_cam/CycleArray.h"

#include <boost/foreach.hpp>

using namespace argus_common;

namespace v4l2_cam
{
	
	CameraManager::CameraManager( const ros::NodeHandle& nh, 
								  image_transport::CameraPublisher& ipub,
								  const std::string& _cameraName )
		: cameraName( _cameraName ), 
		nodeHandle( nh ),
		imagePort( nodeHandle ),
		imagePub( ipub )
	{
		ros::service::waitForService( cameraName + "/set_streaming" );
		setStreamingClient = nodeHandle.serviceClient<SetStreaming>( 
				cameraName + "/set_streaming", true );
		
		imageSub = imagePort.subscribeCamera( cameraName + "/image_raw", 1,
					&CameraManager::ImageCallback, this );
	}
	
	CameraManager::~CameraManager()
	{
// 		DisableCamera();
	}
	
	void CameraManager::ImageCallback( const sensor_msgs::Image::ConstPtr& msg,
									   const sensor_msgs::CameraInfo::ConstPtr& info_msg )
	{
		imagePub.publish( msg, info_msg );
	}
	
	void CameraManager::ValidateConnection()
	{
		if( !setStreamingClient.isValid() )
		{
			ROS_WARN_STREAM( "Lost connection to camera " << cameraName
				<< ". Attempting to reconnect..." );
			setStreamingClient = nodeHandle.serviceClient<SetStreaming>( 
				cameraName + "/set_streaming", true );
		}
	}
	
	void CameraManager::EnableCamera()
	{
// 		ValidateConnection();
		SetStreaming srv;
		srv.request.enableStreaming = true;
		srv.request.numFramesToStream = 0;
		setStreamingClient.call( srv );
	}
	
	void CameraManager::DisableCamera()
	{
// 		ValidateConnection();
		SetStreaming srv;
		srv.request.enableStreaming = false;
		srv.request.numFramesToStream = 0;
		setStreamingClient.call( srv );
	}
	
	CameraArray::CameraArray( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
		: nodeHandle( nh ), privHandle( ph ),
		imagePort( ph )
	{
		
		std::vector<std::string> cameraNames;
		privHandle.getParam( "cameras", cameraNames );
		
		// Have to initialize this first to avoid race condition
		imagePub = imagePort.advertiseCamera( "image_raw", 1 );
		
		BOOST_FOREACH( const std::string& name, cameraNames )
		{
			CameraManager::Ptr manager = 
				std::make_shared<CameraManager>( nodeHandle, imagePub, name );
			registry[ name ] = manager;
		}
		
		dispatchers.SetNumWorkers( registry.size() ); // One per camera
		dispatchers.StartWorkers();
		
		cycleArrayServer = privHandle.advertiseService( "cycle_array",
							&CameraArray::CycleArrayService, this );
		enableCameraServer = privHandle.advertiseService( "enable_camera",
							&CameraArray::EnableCameraService, this );
		disableCameraServer = privHandle.advertiseService( "disable_camera",
							&CameraArray::DisableCameraService, this );
		disableAllServer = privHandle.advertiseService( "disable_all",
							&CameraArray::DisableArrayService, this );
		listCamerasServer = privHandle.advertiseService( "list_cameras",
							&CameraArray::ListCamerasService, this );
	}
	
	CameraArray::~CameraArray()
	{
		dispatchers.StopWorkers();
	}
	
	bool CameraArray::CycleArrayService( CycleArray::Request& req, CycleArray::Response& res )
	{
	
		SetStreaming srv;
		BOOST_FOREACH( CameraRegistry::value_type& item, registry )
		{
			EnableCamera( item.second );
		}
		return true;
	}
	
	bool CameraArray::EnableCameraService( EnableArrayCamera::Request& req,
										   EnableArrayCamera::Response& res )
	{
		try
		{
			CameraManager::Ptr manager = registry.at( req.cameraName );
			EnableCamera( manager );
			return true;
		}
		catch( std::out_of_range e )
		{
			ROS_ERROR_STREAM( "Cannot enable unregistered camera " << req.cameraName );
			return false;
		}
	}
	
	bool CameraArray::DisableCameraService( DisableArrayCamera::Request& req,
											DisableArrayCamera::Response& res )
	{
		try
		{
			CameraManager::Ptr manager = registry.at( req.cameraName );
			DisableCamera( manager );
			return true;
		}
		catch( std::out_of_range e )
		{
			ROS_ERROR_STREAM( "Cannot disable unregistered camera " << req.cameraName );
			return false;
		}
	}
	
	bool CameraArray::DisableArrayService( DisableArray::Request& req,
										   DisableArray::Response& res ) {
		BOOST_FOREACH( CameraRegistry::value_type& item, registry )
		{
			DisableCamera( item.second );
		}
		return true;
	}
	
	bool CameraArray::ListCamerasService( ListArrayCameras::Request& req,
										  ListArrayCameras::Response& res )
	{
		res.cameraNames.reserve( registry.size() );
		BOOST_FOREACH( CameraRegistry::value_type& item, registry )
		{
			res.cameraNames.push_back( item.first );
		}
		return true;
	}
	
	void CameraArray::EnableCamera( CameraManager::Ptr manager )
	{
		WorkerPool::Job job =
			boost::bind( &CameraManager::EnableCamera, manager.get() );
		dispatchers.EnqueueJob( job );
	}
	
	void CameraArray::DisableCamera( CameraManager::Ptr manager )
	{
		WorkerPool::Job job =
			boost::bind( &CameraManager::DisableCamera, manager.get() );
		dispatchers.EnqueueJob( job );
	}
	
}
