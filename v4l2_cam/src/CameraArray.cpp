#include "v4l2_cam/CameraArray.h"
#include "v4l2_cam/SetStreaming.h"
#include "v4l2_cam/CycleArray.h"

#include <boost/foreach.hpp>

using namespace argus_common;

namespace v4l2_cam
{
	
	CameraManager::CameraManager( const ros::NodeHandle& nh, const std::string& _cameraName )
		: cameraName( _cameraName ), nodeHandle( nh )
	{
		setStreamingClient = nodeHandle.serviceClient<SetStreaming>( 
			cameraName + "/set_streaming", true );
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
		ValidateConnection();
		SetStreaming srv;
		srv.request.enableStreaming = true;
		srv.request.numFramesToStream = 0;
		setStreamingClient.call( srv );
	}
	
	void CameraManager::DisableCamera()
	{
		ValidateConnection();
		SetStreaming srv;
		srv.request.enableStreaming = false;
		srv.request.numFramesToStream = 0;
		setStreamingClient.call( srv );
	}
	
	CameraArray::CameraArray( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
		: nodeHandle( nh ), privHandle( ph ),
		cycleCamerasServer( privHandle.advertiseService(
							"cycle_cameras",
							&CameraArray::CycleArrayService,
							this ) )
	{
		
		std::vector<std::string> cameraNames;
		privHandle.getParam( "cameras", cameraNames );
		
		BOOST_FOREACH( const std::string& name, cameraNames )
		{
			CameraManager::Ptr manager = 
				std::make_shared<CameraManager>( nodeHandle, name );
			registry[ name ] = manager;
		}
		
		dispatchers.SetNumWorkers( registry.size() ); // One per camera
		dispatchers.StartWorkers();
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
		SetStreaming srv;
		BOOST_FOREACH( CameraRegistry::value_type& item, registry )
		{
			DisableCamera( item.second );
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
