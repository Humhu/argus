#include "camera_array/CameraArray.h"
#include "camera_array/CycleArray.h"

#include "camplex/SetStreaming.h"
#include "camplex/CameraCalibration.h"

#include "argus_utils/YamlUtils.h"

#include <boost/foreach.hpp>

#include <iostream>
#include <fstream>

using namespace argus_utils;
using namespace camplex;

namespace camera_array
{
	
	CameraArrayMember::CameraArrayMember( const ros::NodeHandle& nh, 
								  image_transport::CameraPublisher& ipub,
								  const std::string& _name )
		: name( _name ), 
		nodeHandle( nh ),
		imagePort( nodeHandle ),
		imagePub( ipub )
	{
		ros::service::waitForService( name + "/set_streaming" );
		setStreamingClient = nodeHandle.serviceClient<SetStreaming>( 
				name + "/set_streaming", true );
		
		imageSub = imagePort.subscribeCamera( name + "/image_raw", 1,
					&CameraArrayMember::ImageCallback, this );
	}
	
	CameraArrayMember::~CameraArrayMember()
	{
// 		DisableCamera();
	}
	
	void CameraArrayMember::ImageCallback( const sensor_msgs::Image::ConstPtr& msg,
									   const sensor_msgs::CameraInfo::ConstPtr& info_msg )
	{
		imagePub.publish( msg, info_msg );
	}
	
	bool CameraArrayMember::Enable()
	{
		return EnableFor( 0 ); // 0 denotes stream forever
	}
	
	bool CameraArrayMember::EnableFor( unsigned int numFrames )
	{
		SetStreaming srv;
		srv.request.enableStreaming = true;
		srv.request.numFramesToStream = numFrames;
		return setStreamingClient.call( srv );
	}
	
	bool CameraArrayMember::Disable()
	{
		SetStreaming srv;
		srv.request.enableStreaming = false;
		srv.request.numFramesToStream = 0; // Doesn't matter, but set it anyways
		return setStreamingClient.call( srv );
	}

	CameraArray::CameraArray( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
		: nodeHandle( nh ), privHandle( ph ), imagePort( ph )
	{
		XmlRpc::XmlRpcValue camerasXml;
		privHandle.getParam( "cameras", camerasXml );
		YAML::Node camerasYaml = XmlToYaml( camerasXml );
		
		ParseConfig( camerasYaml );
		
		// Have to initialize this first to avoid race condition
		imagePub = imagePort.advertiseCamera( "image_raw", 1 );
		
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
	}
	
	CameraArray::~CameraArray()
	{
		dispatchers.StopWorkers();
	}
	
	bool CameraArray::EnableCamera( const std::string& cameraName )
	{
		try
		{
			CameraArrayMember::Ptr manager = registry.at( cameraName );
			manager->Enable(); // TODO
			return true;
		}
		catch( std::out_of_range e )
		{
			ROS_ERROR_STREAM( "Cannot enable unregistered camera " << cameraName );
			return false;
		}
	}
	
	bool CameraArray::DisableCamera( const std::string& cameraName )
	{
		try
		{
			CameraArrayMember::Ptr manager = registry.at( cameraName );
			manager->Disable();
			return true;
		}
		catch( std::out_of_range e )
		{
			ROS_ERROR_STREAM( "Cannot disable unregistered camera " << cameraName );
			return false;
		}
	}
	
	void CameraArray::SaveConfig( const std::string& path )
	{
		XmlRpc::XmlRpcValue camerasXml;
		privHandle.getParam( "cameras", camerasXml );
		YAML::Node camerasYaml = XmlToYaml( camerasXml );
		
		std::ofstream output( path );
		if( !output.is_open() )
		{
			throw std::runtime_error( "Could not create config at " + path );
		}
		output << camerasYaml;
	}
	
	bool CameraArray::CycleArrayService( CycleArray::Request& req, CycleArray::Response& res )
	{
	
		SetStreaming srv;
		BOOST_FOREACH( CameraRegistry::value_type& item, registry )
		{
			item.second->Enable();
		}
		return true;
	}
	
	bool CameraArray::EnableCameraService( EnableArrayCamera::Request& req,
										   EnableArrayCamera::Response& res )
	{
		return EnableCamera( req.cameraName );
	}
	
	bool CameraArray::DisableCameraService( DisableArrayCamera::Request& req,
											DisableArrayCamera::Response& res )
	{
		return DisableCamera( req.cameraName );
	}
	
	bool CameraArray::DisableArrayService( DisableArray::Request& req,
										   DisableArray::Response& res ) {
		BOOST_FOREACH( CameraRegistry::value_type& item, registry )
		{
			item.second->Disable();
		}
		return true;
	}
	
	void CameraArray::ParseConfig( const YAML::Node& config )
	{
		YAML::Node::const_iterator iter;
		for( iter = config.begin(); iter != config.end(); iter++ )
		{
			std::string cameraName = iter->first.as<std::string>();
			PoseSE3 extrinsics;
			GetPoseYaml( iter->second["extrinsics"], extrinsics );
			
			ROS_INFO_STREAM( "Registering camera  " << cameraName 
				<< " with extrinsics " << extrinsics );
			
			CameraArrayMember::Ptr manager = 
				std::make_shared<CameraArrayMember>( nodeHandle, imagePub, cameraName );
			manager->extrinsics = extrinsics;
			registry[ cameraName ] = manager;
			
		}
	}
	
}
