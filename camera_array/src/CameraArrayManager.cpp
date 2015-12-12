#include "camera_array/CameraArrayManager.h"
#include "argus_utils/ParamUtils.h"
#include "camplex/SetStreaming.h" // TODO Move into argus_msgs?
#include <boost/foreach.hpp>

using namespace argus_utils;

namespace camera_array
{
	
CameraArrayManager::CameraArrayManager( const ros::NodeHandle& nh,
                                        const ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph )
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
	
	GetParamDefault<unsigned int>( privHandle, "max_active_cameras", maxNumActive, 1 );
	cameraWorkers.SetNumWorkers( maxNumActive );
	cameraWorkers.StartWorkers();
	
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
		
		ROS_INFO_STREAM( "Registering camera " << name );
		registration.name = name;
		registration.setStreaming = 
			nodeHandle.serviceClient<camplex::SetStreaming>( setStreamingName );
		cameraRegistry[name] = registration;
		
		// HACK to allow forcing streaming off
		currentState.activeCameras.insert( name );
		SetStreaming( name, false );
	}
}

CameraArrayManager::~CameraArrayManager()
{
	cameraWorkers.StopWorkers();
	cameraWorkers.WaitOnJobs();
}

unsigned int CameraArrayManager::MaxActiveCameras() const
{
	return maxNumActive;
}

bool CameraArrayManager::IsActive( const std::string& name ) const
{
	return currentState.activeCameras.count( name ) > 0;
}

const CameraArrayState& CameraArrayManager::GetState() const
{
	return currentState;
}

bool CameraArrayManager::SetStreaming( const std::string& name, bool mode )
{
	
	if( cameraRegistry.count( name ) == 0 ) { return false; }
	
	WriteLock lock( mutex );
	
	// If the current status already matches, return
	CameraRegistration& registration = cameraRegistry[ name ];
	if( IsActive( name ) == mode ) { return true; }
	
	// If we're at capacity, return
	if( mode && currentState.activeCameras.size() >= maxNumActive )
	{
		ROS_WARN_STREAM( "Cannot activate camera " << name << ". Already max num active." );
		return false;
	}
	
	if( mode )
	{
		currentState.inactiveCameras.erase( currentState.inactiveCameras.find( name ) );
		currentState.activeCameras.insert( name );
	}
	else
	{
		currentState.activeCameras.erase( currentState.activeCameras.find( name ) );
		currentState.inactiveCameras.insert( name );
	}
	
	// Call the set streaming service and update the active set
	camplex::SetStreaming srv;
	srv.request.enableStreaming = mode;
	lock.unlock();
	
	// Need this section to be out of lock or else all our service calls will be serial
	if( !registration.setStreaming.call( srv ) )
	{
		ROS_WARN_STREAM( "Could not set streaming for camera " << name );
		return false;
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
	BOOST_FOREACH( const std::string& name, currentState.activeCameras )
	{
		if( ref.count( name ) == 0 ) { toDisable.push_back( name ); }
	}
	
	for( unsigned int i = 0; i < toDisable.size(); i++ )
	{
		RequestSetStreaming( toDisable[i], false );
	}
	for( unsigned int i = 0; i < toActivate.size(); i++ )
	{
		RequestSetStreaming( toActivate[i], true );
	}
	return true;
}

void CameraArrayManager::RequestSetStreaming( const std::string& name, bool mode )
{
	WorkerPool::Job job = boost::bind( &CameraArrayManager::SetStreamingJob,
	                                   this,
	                                   name,
	                                   mode );
	cameraWorkers.EnqueueJob( job );
}

void CameraArrayManager::SetStreamingJob( const std::string& name, bool mode )
{
	if( !SetStreaming( name, mode ) )
	{
		ROS_WARN( "Set streaming job failed." );
	}
}

}
