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
	
	GetParamDefault<unsigned int>( privHandle, "max_num_active", maxNumActive, 1 );
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
		
		SetStreaming( name, false, true );
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
	return activeCameras.count( name ) > 0;
}

const CameraSet& CameraArrayManager::ActiveCameras() const
{
	ReadLock lock( mutex );
	return activeCameras;
}

bool CameraArrayManager::SetStreaming( const std::string& name, bool mode, bool force )
{
	WriteLock lock( mutex );
	
	if( cameraRegistry.count( name ) == 0 ) { return false; }
	
	// If the current status already matches, return
	CameraRegistration& registration = cameraRegistry[ name ];
	if( IsActive( name ) == mode && !force ) { return true; }
	
	if( mode && activeCameras.size() >= maxNumActive )
	{
		ROS_ERROR_STREAM( "Cannot activate camera " << name << ". Already max num active." );
		return false;
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
	
	lock.lock();
	if( mode )
	{
		activeCameras.insert( name );
	}
	else
	{
		CameraSet::iterator iter = activeCameras.find( name );
		if( iter != activeCameras.end() ) { activeCameras.erase( iter ); }
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
		RequestSwitchStreaming( toDisable[i], toActivate[i] );
	}
	for( unsigned int i = mid; i < toActivate.size(); i++ )
	{
		RequestSetStreaming( toActivate[i], true );
	}
	for( unsigned int i = mid; i < toDisable.size(); i++ )
	{
		RequestSetStreaming( toDisable[i], false );
	}
	cameraWorkers.WaitOnJobs();
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

void CameraArrayManager::SetStreamingJob( const std::string& name, bool mode )
{
	if( !SetStreaming( name, mode ) )
	{
		ROS_WARN( "Set streaming job failed." );
	}
}

void CameraArrayManager::SwitchStreamingJob( const std::string& toDisable, 
                                             const std::string& toEnable )
{
	if( !SwitchStreaming( toDisable, toEnable ) )
	{
		ROS_WARN( "Switch streaming job failed." );
	}
}

void CameraArrayManager::RequestSetStreaming( const std::string& name, bool mode )
{
	WorkerPool::Job job = boost::bind( &CameraArrayManager::SetStreamingJob,
	                                   this,
	                                   name,
	                                   mode );
	cameraWorkers.EnqueueJob( job );
}

void CameraArrayManager::RequestSwitchStreaming( const std::string& toDisable,
                                                 const std::string& toEnable )
{
	WorkerPool::Job job = boost::bind( &CameraArrayManager::SwitchStreamingJob,
	                                   this,
	                                   toDisable,
	                                   toEnable );
	cameraWorkers.EnqueueJob( job );
}

}
