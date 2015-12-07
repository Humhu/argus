#include "camera_array/CameraArrayManager.h"
#include "argus_utils/ParamUtils.h"
#include "camplex/SetStreaming.h" // TODO Move into argus_msgs?
#include <boost/foreach.hpp>

using namespace argus_utils;

namespace camera_array
{
	
CameraArrayManager::CameraArrayManager( const ros::NodeHandle& nh,
                                        const ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), extrinsicsManager( lookupInterface )
{
	std::vector<std::string> members;
	if( !ph.getParam( "cameras", members ) )
	{
		ROS_ERROR_STREAM( "Must specify member cameras." );
		exit( -1 );
	}
	
	if( !ph.getParam( "body_frame", bodyFrame ) ||
		!ph.getParam( "reference_frame", referenceFrame ) )
	{
		ROS_ERROR_STREAM( "Must specify reference and body frame IDs." );
		exit( -1 );
	}
	
	GetParamDefault<unsigned int>( nodeHandle, "max_num_active", maxNumActive, 1 );
	
	BOOST_FOREACH( const std::string& name, members )
	{
		CameraRegistration registration;
		
		std::string ns;
		if( !lookupInterface.ReadNamespace( name, ns ) )
		{
			ROS_ERROR_STREAM( "Could not read member namespace." );
			exit( -1 );
		}
		
		std::string setStreamingName = ns + "set_streaming";
		if( !ros::service::waitForService( setStreamingName, ros::Duration( 5.0 ) ) )
		{
			ROS_ERROR_STREAM( "Could not find service " << setStreamingName );
			exit( -1 );
		}
		
		if( !extrinsicsManager.ReadMemberInformation( name ) )
		{
			ROS_ERROR_STREAM( "Could not get extrinsics for camera " << name );
			exit( -1 );
		}
		
		const std::string camRefFrame = extrinsicsManager.GetReferenceFrame( name );
		if( camRefFrame != bodyFrame )
		{
			ROS_ERROR_STREAM( "Reference frame for camera " << name 
				<< " of " << camRefFrame 
				<< " does not match body frame " << bodyFrame );
			exit( -1 );
		}
		
		ROS_INFO_STREAM( "Registering camera " << name );
		registration.name = name;
		registration.setStreaming = 
			nodeHandle.serviceClient<camplex::SetStreaming>( setStreamingName );
		cameraRegistry[name] = registration;
	}
	
}

unsigned int CameraArrayManager::MaxActiveCameras() const
{
	return maxNumActive;
}

bool CameraArrayManager::IsActive( const std::string& name ) const
{
	return activeCameras.count( name ) > 0;
}

const CameraSet& CameraArrayManager::ActiveCameras() const
{
	return activeCameras;
}

bool CameraArrayManager::SetStreaming( const std::string& name, bool mode )
{
	if( cameraRegistry.count( name ) == 0 ) { return false; }
	
	// If the current status already matches, return
	CameraRegistration& registration = cameraRegistry[ name ];
	if( IsActive( name ) == mode ) { return true; }
	
	if( mode && activeCameras.size() >= maxNumActive )
	{
		ROS_ERROR_STREAM( "Cannot activate cameras. Already max num active." );
		return false;
	}
	
	// Call the set streaming service and update the active set
	camplex::SetStreaming srv;
	srv.request.enableStreaming = mode;
	if( !registration.setStreaming.call( srv ) )
	{
		ROS_WARN_STREAM( "Could not set streaming for camera " << name );
		return false;
	}
	
	if( mode )
	{
		activeCameras.insert( name );
	}
	else
	{
		activeCameras.erase( activeCameras.find( name ) );
	}
	
	return true;
}

bool CameraArrayManager::SetActiveCameras( const CameraSet& ref )
{
	if( ref.size() > maxNumActive )
	{
		ROS_ERROR_STREAM( "Cannot set " << ref.size() << " active cameras for limit "
		    << maxNumActive );
		return false;
	}
	
	std::vector<std::string> toActivate;
	BOOST_FOREACH( const std::string& name, ref )
	{
		if( !IsActive( name ) ) { toActivate.push_back( name ); }
	}
	
	std::vector<std::string> toDisable;
	BOOST_FOREACH( const std::string& name, activeCameras )
	{
		if( ref.count( name ) == 0 ) { toDisable.push_back( name ); }
	}
	
	unsigned int mid = std::min( toActivate.size(), toDisable.size() );
	for( unsigned int i = 0; i < mid; i++ )
	{
		if( !SwitchStreaming( toDisable[i], toActivate[i] ) ) { return false; }
	}
	for( unsigned int i = mid; i < toActivate.size(); i++ )
	{
		if( !SetStreaming( toActivate[i], true ) ) { return false; }
	}
	for( unsigned int i = mid; i < toDisable.size(); i++ )
	{
		if( !SetStreaming( toDisable[i], false ) ) { return false; }
	}
	return true;
}

bool CameraArrayManager::SwitchStreaming( const std::string& toDisable, 
                                   const std::string& toEnable )
{
	if( toDisable == toEnable ) { return true; }
	
	if( !SetStreaming( toDisable, false ) ) 
	{ 
		ROS_WARN_STREAM( "Could not switch cameras " << toDisable << " and "
		    << toEnable << " because disable failed." );
		return false; 
	}
	
	if( !SetStreaming( toEnable, true ) )
	{
		ROS_WARN_STREAM( "Could not switch cameras " << toDisable << " and "
		    << toEnable << " because enable failed." );
		return false;
	}
	return true;
}


}
