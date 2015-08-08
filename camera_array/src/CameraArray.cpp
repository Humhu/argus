#include "camera_array/CameraArray.h"

#include "camplex/CameraCalibration.h"
#include "camplex/GetCameraInfo.h"

#include "argus_utils/YamlUtils.h"

#include <boost/foreach.hpp>

#include <iostream>
#include <fstream>

using namespace argus_utils;
using namespace camplex;

namespace camera_array
{

	CameraArray::CameraArray( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
		: nodeHandle( nh ), privHandle( ph ), 
		extrinsicsManager( nodeHandle, privHandle )
	{
		// NOTE CameraArray file doubles as camera array member declarations
		std::string extrinsicsPath;
		ph.getParam( "extrinsics_info_url", extrinsicsPath );
		if( !extrinsicsManager.LoadCameraArrayInfo( extrinsicsPath ) )
		{
			ROS_ERROR_STREAM( "Could not load extrinsics from " << extrinsicsPath );
			exit( -1 );
		}
		
		CameraArrayInfo extrinsics = extrinsicsManager.GetCameraArrayInfo();
		BOOST_FOREACH( const std::string& name, extrinsics.cameraNames )
		{
			RegisterCamera( name );
		}
		
		// TODO Caused stack smashing, not sure why yet
		// Have to initialize this first to avoid race condition
// 		dispatchers.SetNumWorkers( registry.size() ); // One per camera
// 		dispatchers.StartWorkers();
		
		setStreamingServer = privHandle.advertiseService( "set_stream",
							&CameraArray::SetStreamService, this );
		captureFramesServer = privHandle.advertiseService( "capture_frames",
							&CameraArray::CaptureFramesService, this );
	}
	
	CameraArray::~CameraArray()
	{
// 		dispatchers.StopWorkers();
	}
	
	bool CameraArray::CallSetStreaming( const std::string& cameraName, bool stream )
	{
		try 
		{ 
			return registry.at( cameraName ).CallSetStreaming( stream );
		}
		catch( std::out_of_range e )
		{
			ROS_ERROR_STREAM( "Cannot set unregistered camera " << cameraName );
			return false;
		}
	}
	
	bool CameraArray::CallCaptureFrames( const std::string& cameraName, unsigned int num )
	{
		try
		{
			return registry.at( cameraName ).CallCaptureFrames( num );
		}
		catch( std::out_of_range e )
		{
			ROS_ERROR_STREAM( "Cannot capture from unregistered camera " << cameraName );
			return false;
		}
	}
	
	bool CameraArray::SetStreamService( SetStreaming::Request& req, 
										SetStreaming::Response& res )
	{
		bool success = true;
		BOOST_FOREACH( CameraRegistry::value_type& item, registry )
		{
			success &= item.second.CallSetStreaming( req.enableStreaming );
		}
		return success;
	}
	
	bool CameraArray::CaptureFramesService( CaptureFrames::Request& req,
											CaptureFrames::Response& res )
	{
		bool success = true;
		BOOST_FOREACH( CameraRegistry::value_type& item, registry )
		{
			success &= item.second.CallCaptureFrames( req.numToCapture );
		}
		return success;
	}
	
	CameraArray::CameraRegistration::CameraRegistration( ros::NodeHandle& nh,
														 const std::string& cameraName )
	{
		ros::service::waitForService( cameraName + "/set_streaming" );
		ros::service::waitForService( cameraName + "/capture_frames" );
		setStreamingClient = 
			nh.serviceClient<SetStreaming>( cameraName + "/set_streaming" );
		captureFramesClient = 
			nh.serviceClient<CaptureFrames>( cameraName + "/capture_frames" );
	}
	
	bool CameraArray::CameraRegistration::CallSetStreaming( bool enable )
	{
		SetStreaming ss;
		ss.request.enableStreaming = enable;
		return setStreamingClient.call( ss );
	}
	
	bool CameraArray::CameraRegistration::CallCaptureFrames( unsigned int num )
	{
		CaptureFrames cf;
		cf.request.numToCapture = num;
		return captureFramesClient.call( cf );
	}
	
	void CameraArray::RegisterCamera( const std::string& cameraName )
	{
		
		if( registry.count( cameraName ) > 0 )
		{
			ROS_ERROR_STREAM( "Already have registration for camera " << cameraName );
			return;
		}
		
		ROS_INFO_STREAM( "Registering camera " << cameraName );
		
		CameraRegistration reg( nodeHandle, cameraName );
		registry.insert( std::make_pair( cameraName, reg ) );
			
	}
	
}
